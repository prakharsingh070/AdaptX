# Tracking

## Lifecycle

Detection -> data association -> track creation -> track update -> track prediction -> track termination.

## Track State

A tracked object should conceptually contain:

- `track_id`
- class
- position
- velocity
- acceleration
- heading
- confidence
- age
- last-seen timestamp
- prediction

## Requirements

Define association behavior, initialization, missed detections, occlusion handling, identity stability, termination rules, and coordinate-frame conventions. Tracking outputs must be timestamped and usable by both prediction and risk modules.
