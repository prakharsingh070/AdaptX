# Data Schema

These examples establish shared concepts. Implementations must validate and version schemas rather than silently inventing incompatible formats.

## Detected or Tracked Object

```json
{
  "id": 12,
  "type": "pedestrian",
  "position": {"x": 12.4, "y": -3.1, "z": 0.0},
  "velocity": 1.8,
  "confidence": 0.91,
  "risk": 82,
  "uncertainty": 0.17,
  "timestamp": "2026-01-01T00:00:00Z"
}
```

Values must include units or be defined by the contract. `id` is stable for the track lifetime, not necessarily a global identity.

## 2.5D Map Cell

```json
{
  "x": 10,
  "y": 20,
  "occupancy": 0.92,
  "height": 1.7,
  "risk": 84,
  "resolution": 0.10,
  "confidence": 0.89,
  "timestamp": "2026-01-01T00:00:00Z"
}
```

## General Rules

Define coordinate frame, units, timestamp semantics, nullability, confidence ranges, risk ranges, and schema version for each public model.
