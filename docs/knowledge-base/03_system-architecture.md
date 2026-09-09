# System Architecture

## High-Level Layers

Sensor layer -> perception layer -> world representation -> risk layer -> prediction layer -> adaptive-resolution layer -> evaluation layer -> API -> dashboard.

## Sensor Layer

Inputs: LiDAR, optional camera, and CARLA sensor streams.

Outputs: timestamped point-cloud frames and sensor metadata.

## Perception Layer

Input: point cloud.

Outputs: detected objects with class, position, dimensions, and confidence.

## Tracking Layer

Input: detections across frames.

Outputs: persistent object IDs, velocity, acceleration, heading, and tracking confidence.

## Mapping Layer

Inputs: point cloud, detections, and ego pose.

Output: 2.5D occupancy representation.

## Risk Layer

Inputs: objects, velocity, distance, ego trajectory, and uncertainty.

Output: spatial risk field.

## Prediction Layer

Input: tracked object history.

Outputs: predicted trajectories, prediction confidence, and future risk.

## Adaptive Resolution

Inputs: current risk, predicted risk, uncertainty, object density, distance, ego trajectory, and predicted object trajectories.

Output: spatial resolution field and adaptive occupancy map.

## Evaluation Layer

Inputs: fixed-baseline results and ADAPT-X results.

Outputs: FPS, latency, memory, compute workload, and perception metrics.

## Boundary Rule

Perception modules must not depend on dashboard components. The dashboard consumes structured backend data through stable contracts.
