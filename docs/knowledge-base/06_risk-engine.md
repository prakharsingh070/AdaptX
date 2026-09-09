# Risk Engine

## Purpose

The risk engine estimates the relative importance of spatial regions for perception and future planning. It is a research component whose formulation must be evaluated experimentally.

## Conceptual Inputs

- distance and proximity
- relative velocity
- object class
- time-to-collision
- predicted trajectory
- ego trajectory
- uncertainty
- object density
- object importance or priority

## Conceptual Model

`Risk(x, y) = f(proximity, relative motion, TTC, trajectory overlap, uncertainty, object importance)`

This is a conceptual model, not a mandated equation. Any implemented formulation must document assumptions, normalization, bounds, thresholds, and validation results.

## Outputs

The engine should produce a spatial risk field, per-object risk where applicable, contributing factors, timestamps, and enough metadata for event replay and dashboard explanation.
