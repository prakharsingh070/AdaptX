"""Structured application logging.

Two formats are supported:

``text``
    Human-readable single line, used by default in development.
``json``
    One JSON object per line, suitable for log shipping in containers.

Both always carry timestamp, level, logger name and message. Additional
contextual fields are attached per call via ``extra={"context": {...}}``.

Logging discipline
------------------
High-frequency paths (per-frame ingest, per-frame telemetry) log at ``DEBUG``
only. Never log per-point or per-cell inside processing loops.
"""

from __future__ import annotations

import json
import logging
import sys
from datetime import UTC, datetime
from typing import Any

_CONFIGURED = False

#: Attributes present on every LogRecord; anything else is treated as context.
#: ``color_message`` is uvicorn's own extra, which would otherwise be repeated
#: as context on every server log line.
_RESERVED = frozenset(logging.LogRecord("", 0, "", 0, "", (), None).__dict__) | {
    "message",
    "asctime",
    "taskName",
    "color_message",
}


def _timestamp(record: logging.LogRecord) -> str:
    return datetime.fromtimestamp(record.created, tz=UTC).isoformat(timespec="milliseconds")


class JsonFormatter(logging.Formatter):
    """Render a log record as a single JSON line."""

    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, Any] = {
            "timestamp": _timestamp(record),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }
        context = _extract_context(record)
        if context:
            payload["context"] = context
        if record.exc_info:
            payload["exception"] = self.formatException(record.exc_info)
        return json.dumps(payload, default=str)


class TextFormatter(logging.Formatter):
    """Render a log record as a readable single line with optional context."""

    def format(self, record: logging.LogRecord) -> str:
        base = (
            f"{_timestamp(record)} | {record.levelname:<8} | {record.name} | {record.getMessage()}"
        )
        context = _extract_context(record)
        if context:
            rendered = " ".join(f"{key}={value}" for key, value in context.items())
            base = f"{base} | {rendered}"
        if record.exc_info:
            base = f"{base}\n{self.formatException(record.exc_info)}"
        return base


def _extract_context(record: logging.LogRecord) -> dict[str, Any]:
    """Collect structured fields from ``extra={"context": ...}`` and ad-hoc extras."""
    context: dict[str, Any] = {}
    explicit = getattr(record, "context", None)
    if isinstance(explicit, dict):
        context.update(explicit)
    for key, value in record.__dict__.items():
        if key not in _RESERVED and key != "context":
            context[key] = value
    return context


def configure_logging(level: str = "INFO", *, json_format: bool = False) -> None:
    """Install ADAPT-X logging handlers on the root logger.

    Safe to call more than once; repeated calls only update the level and
    formatter rather than stacking handlers.
    """
    global _CONFIGURED

    formatter: logging.Formatter = JsonFormatter() if json_format else TextFormatter()
    root = logging.getLogger()
    root.setLevel(level.upper())

    if not _CONFIGURED:
        handler = logging.StreamHandler(stream=sys.stdout)
        handler.set_name("adaptx")
        root.addHandler(handler)
        _CONFIGURED = True

    for existing in root.handlers:
        if existing.get_name() == "adaptx":
            existing.setFormatter(formatter)
            existing.setLevel(level.upper())

    # Uvicorn installs its own handlers; route them through ours instead.
    for name in ("uvicorn", "uvicorn.error", "uvicorn.access"):
        uvicorn_logger = logging.getLogger(name)
        uvicorn_logger.handlers.clear()
        uvicorn_logger.propagate = True


def get_logger(name: str) -> logging.Logger:
    """Return a module logger. Use ``__name__`` at the call site."""
    return logging.getLogger(name)
