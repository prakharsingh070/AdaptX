# Event Replay

## Purpose

Event replay explains how system state changed over time and why the adaptive map changed resolution.

## Event Sequence Example

Object detected -> object tracked -> risk increased -> prediction updated -> predicted collision zone created -> map resolution increased -> risk decreased -> map resolution reduced.

## Event Fields

At minimum, an event should contain:

- timestamp
- event type
- source module
- object ID or region, when applicable
- relevant before and after values
- reason or contributing factors
- scenario or frame identifier

Example event types include `object_detected`, `risk_changed`, `prediction_updated`, `resolution_change`, and `benchmark_sample`.
