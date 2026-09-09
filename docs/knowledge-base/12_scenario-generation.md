# Scenario Generation

## Supported Parameters

Scenario generation should support:

- road type
- traffic density
- number of vehicles
- number of pedestrians
- number of cyclists
- obstacle presence
- weather
- time of day
- ego-vehicle speed
- pedestrian crossing
- sudden vehicle cut-in
- stationary obstacle
- emergency scenario

## Example Scenarios

### Normal

Ten vehicles, no pedestrians, clear weather.

### Pedestrian Crossing

Vehicles and pedestrians with one pedestrian crossing the ego trajectory.

### Cut-In

A vehicle changes lane suddenly.

### Obstacle

A stationary obstacle appears in the driving region.

### Heavy Traffic

A high number of vehicles create dense tracks and occlusions.

### Bad Weather

Rain, fog, or night conditions increase sensing difficulty.

## Reproducibility

`random seed + scenario configuration = reproducible experiment`. Store the complete configuration with every run.
