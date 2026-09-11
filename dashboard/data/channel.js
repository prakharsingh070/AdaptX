// A reconnecting WebSocket client with a stale indicator. Used for both
// /ws/telemetry and /ws/scene; the two channels differ only in payload.
//
// State reported to listeners:
//   connected   - socket open
//   lastMessageAt - wall-clock time of the last message received
//   stale       - no message for longer than staleAfterMs while connected,
//                 or disconnected with a previous message still displayed
//   last        - the last envelope's data (kept while stale, marked stale)

import { wsBase } from "./api.js";

export class Channel {
  constructor(path, { staleAfterMs = 3000, reconnectMs = 1500, onMessage = null, onState = null } = {}) {
    this.path = path;
    this.staleAfterMs = staleAfterMs;
    this.reconnectMs = reconnectMs;
    this.onMessage = onMessage;
    this.onState = onState;
    this.socket = null;
    this.connected = false;
    this.lastMessageAt = null;
    this.lastSequence = null;
    this.last = null;
    this.closedByUser = false;
    this.timer = null;
    this.staleTimer = null;
  }

  get stale() {
    if (this.lastMessageAt === null) return !this.connected;
    if (!this.connected) return true;
    return Date.now() - this.lastMessageAt > this.staleAfterMs;
  }

  state() {
    return {
      connected: this.connected,
      stale: this.stale,
      lastMessageAt: this.lastMessageAt,
      lastSequence: this.lastSequence,
    };
  }

  open() {
    this.closedByUser = false;
    this._connect();
    if (!this.staleTimer) {
      this.staleTimer = setInterval(() => this._emitState(), 500);
    }
  }

  close() {
    this.closedByUser = true;
    if (this.timer) clearTimeout(this.timer);
    if (this.staleTimer) clearInterval(this.staleTimer);
    this.staleTimer = null;
    if (this.socket) this.socket.close();
  }

  _connect() {
    let socket;
    try {
      socket = new WebSocket(`${wsBase()}${this.path}`);
    } catch (error) {
      this._scheduleReconnect();
      return;
    }
    this.socket = socket;
    socket.onopen = () => {
      this.connected = true;
      this._emitState();
    };
    socket.onmessage = (event) => {
      let envelope;
      try {
        envelope = JSON.parse(event.data);
      } catch {
        return; // a malformed frame is dropped, never rendered
      }
      this.lastMessageAt = Date.now();
      this.lastSequence = envelope.sequence ?? this.lastSequence;
      if (envelope.type !== "hello") this.last = envelope;
      if (this.onMessage) this.onMessage(envelope);
      this._emitState();
    };
    socket.onclose = () => {
      this.connected = false;
      this._emitState();
      if (!this.closedByUser) this._scheduleReconnect();
    };
    socket.onerror = () => {
      // onclose follows; nothing to do here
    };
  }

  _scheduleReconnect() {
    if (this.timer) clearTimeout(this.timer);
    this.timer = setTimeout(() => this._connect(), this.reconnectMs);
  }

  _emitState() {
    if (this.onState) this.onState(this.state());
  }
}
