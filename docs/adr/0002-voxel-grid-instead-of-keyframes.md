# 0002. Voxel grid instead of keyframes for point cloud downsampling

- Status: Accepted
- Date: 2026-10-27

## Context

The Pi froze during scanning. Every scan (~500 points at ~10 Hz) was appended to a list that never shrank, and the full list was converted to `PointCloud2` and published on every scan, so each scan cost more than the last. The user often scans from one spot for a long time before moving. The third dimension comes from **tilting** the 2D LiDAR, not from walking.

## Decision

Downsample with a voxel grid in `PointCloudAccumulator`: space is divided into cubes of edge `voxel_size` (default 0.05 m, ROS parameter), key = `floor(coord / voxel_size)` per axis, and only the first point per cube is kept.

## Alternatives considered

- **Keyframes** (store a scan only after the device moved a minimum distance): would discard exactly the data produced by standing still and tilting, which is the main use case.
- **Minimum distance to all stored points:** same idea as the voxel grid, but each check gets slower as the cloud grows. The grid is an O(1) dictionary lookup.
- **OctoMap (`octomap_server`):** standard ROS solution with free-space modelling, but heavier for the Pi and more complexity than needed now.

## Consequences

- Cloud size is bounded by the scanned surface area, not scan time (simulated: 800 points after 1 min standing still vs. ~432,000 before).
- Resolution below the LiDAR's accuracy (a few cm) brings little; 3–5 cm is a sensible range.
- Only meaningful in a consistent world frame: as long as points are not translated (see `progress.md`), walking still smears the cloud, and yaw drift creates extra voxels.
- The full cloud is still republished every scan; a lower publish rate may be needed later.
