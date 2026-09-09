# LiDAR Knowledge

## Point Cloud

A point may contain:

- `x`, `y`, `z` coordinates
- intensity or reflectivity
- timestamp
- optional return or ring metadata

## Coordinate Systems

Document transformations between:

- LiDAR frame
- ego-vehicle frame
- world frame
- camera frame, when a camera is used

Every transformation must define its convention, units, timestamp, and calibration source.

## Processing Pipeline

Raw cloud -> region-of-interest filtering -> voxelization or downsampling -> ground segmentation -> noise filtering -> object processing.

## Important Concepts

The implementation should account for voxel grids, point density, downsampling, clustering, ground segmentation, coordinate transformations, sensor calibration, and timestamp alignment. Processing choices must be configurable and tested against representative data.
