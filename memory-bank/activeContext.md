# Active Context

Last updated: 2026-11-04

## Current focus

Getting the whole pipeline running on the Pi again after a long break, with a point cloud that no longer freezes the Pi.

Preparing a second IMU. Plan by the user, nothing implemented yet: an LSM9DS1 (9DOF) tracks the device, the MPU6050 (6DOF) only measures the LiDAR pitch, because a motor will pitch the LiDAR up and down independently of the device.

## Recent changes (2026-11-04)

- MPU6050-related identifiers in `fusion_calculation_node.py` and `fusion_launch.py` renamed to `six_dof_imu_*` (parse and attribute check off-device, not run on the Pi). Topics, frames and generic helpers such as `ComplementaryFilter` keep their names.

## Recent changes (2026-10-27)

- Repo cleanup: colcon output removed from `src/`, `.gitmodules` added, unused MPU9250 submodule link removed.
- Fixed: syntax error in the fusion node, slam_toolbox config filename, config not installed, YAML missing `ros__parameters`.
- README added.
- Fusion node split into ROS-free modules (behaviour verified identical off-device).
- Voxel grid downsampling with parameter `voxel_size` (logic tested off-device).
- Agent docs added: this memory bank, `docs/adr/`, `AGENTS.md`.
- Scan points are now translated by the device position after rotation (tested off-device).
- Slam pose subscription fixed: `/pose` as `PoseWithCovarianceStamped` (verified in slam_toolbox source, not yet on the Pi).
- Headless VNC fixed on the Pi: `WaylandEnable=false` was missing from `/etc/gdm3/custom.conf` there (see `progress.md`).
- Branch `raspi_remote` (one commit, `25f432e`) is **abandoned**; work continues on `main`. The Pi may still have it checked out.

## Next steps

1. Pi: restore internet (see Raspberry Pi section in `progress.md`), `git checkout main && git pull`.
2. Pi: `colcon build`, launch, confirm everything starts. Check `ros2 param get /slam_toolbox max_laser_range` = `12.0` and `ros2 param get /fusion_node voxel_size`.
3. Confirm `/pose` arrives (`ros2 topic echo /pose`) and the cloud follows the device when walking.
4. Then tackle the circular TF.

## Open decisions

- Publishing strategy if the cloud is still too slow: timer at 1–2 Hz vs. publish only new points with RViz decay time.
- Second IMU: where the device position comes from (an IMU alone gives orientation, not a usable position), how `base_link → laser` becomes dynamic, and whether `/imu`, `/imu_fused`, `/imu_pose` get renamed. `/imu` is hardcoded in the MPU6050 driver, so a second driver must not publish on it.
- LSM9DS1 driver: candidate `nm47/lsm9ds1_handler` (source read, not built or run; last push 2023-03). Publishes `/lsm9ds1/telemetry` (`Imu`, gyro in deg/s). Not usable unchanged, would need a fork ([ADR 0004](../docs/adr/0004-drivers-as-git-submodules.md)): magnetometer not published, multi-byte `read_mag` probably wrong (no `0x80` in the sub-address), `calibrate_accelgyro()` empty, header stamp and `frame_id` never set, wrong sensitivity for `gyro_scale: 0`.
- Replacing slam_toolbox (goal is a 3D voxel map, not a 2D map; would supersede ADR 0005). Planned order, not implemented: 1a remove it, device fixed, motor pitching, MPU6050 pitch only (position `(0, 0, 0)`, roll and yaw 0), measure the error at a flat wall. 1b add the LSM9DS1, handheld from a fixed spot. 2 get the position from KISS-ICP (`prbonn/kiss-icp`). Read from its source, untested here: ROS 2 wrapper via `colcon build`, input `PointCloud2`, no IMU input, output `kiss/odometry`. It would need one assembled cloud per nod sweep. Jazzy build and Pi 5 performance unknown.
