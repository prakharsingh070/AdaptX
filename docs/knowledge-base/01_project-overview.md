# ADAPT-X Project Overview

## What Is ADAPT-X?

ADAPT-X stands for Adaptive Dynamic Perception and Tracking. It is a LiDAR-based intelligent perception framework that allocates computational resources according to environmental risk.

Traditional perception can maintain a fixed spatial resolution across the sensing region. ADAPT-X treats the environment as a dynamic risk field: important regions receive greater spatial detail while less important regions receive lower detail.

## Core Idea

The system continuously asks:

- Where does the vehicle need the most perception detail now?
- Where will it need more perception detail in the near future?

## Example

- Empty road: coarse representation.
- Approaching vehicle: increased resolution.
- Pedestrian near the predicted vehicle path: high resolution.
- Predicted collision zone: maximum refinement.
- Pedestrian moves away: resolution decreases.

## Expected Benefit

The project investigates whether risk-aware adaptive perception can reduce unnecessary computation while preserving or improving perception quality in safety-critical regions. This benefit must be demonstrated experimentally, not assumed.
