# Product Context

## Why

A 2D LiDAR alone only sees one flat slice of a room. Combining it with an IMU turns a cheap 2D sensor into a handheld 3D scanner.

## How scanning is meant to work

1. Start the launch file on the Pi. Keep the device flat and still while the IMU calibrates.
2. Sweep and tilt the scanner by hand. Each 2D scan is rotated by the current orientation and added to the point cloud.
3. Optionally walk to another spot and continue. (Not supported yet: points are only rotated, not translated. See `progress.md`.)
4. Watch the cloud grow live in RViz on the attached monitor.

## User experience goals

- The cloud must stay responsive for the whole scan. The last attempt froze the Pi because the cloud grew without limit (fixed with a voxel grid, see [ADR 0002](../docs/adr/0002-voxel-grid-instead-of-keyframes.md)).
- Standing still must not make the cloud grow.
