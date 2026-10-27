# System Patterns

## Nodes and data flow

```
RPLidar C1 ──/scan──────────┬──────────────► slam_toolbox ──/map, TF map→odom
                            │                     │
                            ▼                     │ /slam_toolbox/pose (unverified, see progress.md)
MPU6050 ──/imu──► fusion_calculation_node ◄───────┘
                     ├── /imu_fused   (Imu with orientation)
                     ├── /imu_pose    (PoseStamped)
                     ├── /pointcloud  (PointCloud2, voxel-downsampled, frame odom)
                     └── TF odom→base_link
```

`launch/fusion_launch.py` starts: rplidar driver (`rplidar_c1_launch.py`), mpu6050 driver, fusion node (`name='fusion_node'`, parameter `voxel_size`), slam_toolbox (`sync_slam_toolbox_node`, `config/mapper_params_online.yaml`), RViz if `use_rviz:=true`.

## TF tree

| Transform | Source |
|---|---|
| `map → odom` | slam_toolbox |
| `odom → base_link` | fusion node: rotation from IMU, translation from slam_toolbox pose |
| `base_link → laser` | fusion node, static identity (LiDAR assumed exactly on the IMU) |

Known flaw: slam_toolbox consumes `odom → base_link`, and the fusion node builds that transform from slam_toolbox's own pose. See `progress.md`.

## Package structure (`src/imu_lidar_sensor_fusion/imu_lidar_sensor_fusion/`)

| Module | Role | ROS-free |
|---|---|---|
| `fusion_calculation_node.py` | ROS wiring only: subscriptions, publishers, TF, callbacks | no |
| `orientation_filter.py` | `ComplementaryFilter.update(gyro_deg_s, accel, dt)` → quaternion | yes |
| `scan_projection.py` | `scan_to_points(scan, orientation)`; WIP `restrict_scan_by_angle` (unused) | yes |
| `point_cloud_accumulator.py` | `PointCloudAccumulator(voxel_size)`: one point per voxel | yes |
| `geometry.py` | quaternion multiply, rotate vector, euler → quaternion | yes |
| `ros_messages.py` | builds Imu, Pose, TF, PointCloud2 messages | no |

Quaternions are tuples `(x, y, z, w)` throughout. Decision: [ADR 0003](../docs/adr/0003-split-fusion-node-into-ros-free-modules.md).

## Key algorithms

- **Orientation:** complementary filter, `alpha = 0.9`. Roll/pitch = gyro integration corrected by accelerometer tilt. Yaw = gyro integration only (MPU6050 has no magnetometer), so yaw drifts. `dt` comes from wall-clock time between IMU callbacks (0.01 s for the first message).
- **Scan projection:** polar → XY in the laser plane (z = 0), drop ranges outside `(range_min, range_max)`, rotate by the latest IMU orientation. No translation.
- **Downsampling:** voxel grid, key = `floor(coord / voxel_size)` per axis, first point per voxel wins. Decision: [ADR 0002](../docs/adr/0002-voxel-grid-instead-of-keyframes.md).
- **Publishing:** the full cloud is converted and published on every scan.

## Existing choices without a recorded rationale

ROS 2, complementary filter, RPLidar C1, MPU6050 predate the ADRs. Their reasons were never written down. Ask the user before writing ADRs for them.
