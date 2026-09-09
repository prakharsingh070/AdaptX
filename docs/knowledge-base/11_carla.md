# CARLA Integration

## Role

CARLA is the controlled, configurable simulation environment for testing ADAPT-X. It is not the project itself.

CARLA provides:

- roads and intersections
- vehicles
- pedestrians and cyclists
- obstacles and traffic
- weather and lighting
- sensor configurations

## Data Flow

CARLA world -> actors and environment -> sensors -> LiDAR and optional camera/GPS -> ADAPT-X -> benchmark and dashboard.

## Requirements

Document CARLA version, world and map configuration, synchronous/asynchronous mode, fixed timestep, sensor transforms, traffic configuration, random seeds, and cleanup behavior. Scenarios must be repeatable when supplied the same configuration and seed.
