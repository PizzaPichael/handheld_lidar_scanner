# Progress

Summary of the current state, not a log. Move items between sections as they change.

## What works

Verified off-device only (no ROS on the dev machine). Nothing below has been run on the Pi since 2026-10-27:

- Orientation filter, scan projection, quaternion math: identical output to the pre-refactor code (randomised comparison, 1e-12).
- Voxel grid: simulated 1 min of standing still in a 5 × 5 m room → cloud stays at 800 points (≈432,000 without voxel grid), ~1 ms per scan on the laptop.

## Encountered problems

Observed, cause known or suspected, not fixed.

### Point cloud / fusion
- **Points only rotated, not translated.** `scan_to_points` ignores position, so walking around stacks all scans at the origin.
- **SLAM pose probably never arrives.** Node subscribes to `/slam_toolbox/pose` as `PoseStamped`; slam_toolbox likely publishes `/pose` as `PoseWithCovarianceStamped`. Unverified. If true, `slam_position` stays `(0, 0, 0)`.
- **Circular TF.** slam_toolbox needs `odom → base_link`; the fusion node builds it from slam_toolbox's pose.
- **Yaw drift.** Gyro-only yaw, no magnetometer. Idea from the abandoned `raspi_remote` branch: ICM-20948 (has a magnetometer).

### Raspberry Pi
- **No internet.** WLAN `MirTutAllesWLan-2.4` connected, static IPv4 `192.168.2.198`, but "Destination Host Unreachable" from its own address. Suspected: gateway still from an older `192.168.0.x` network. Check `ip route` and `nmcli con show "MirTutAllesWLan-2.4" | grep ipv4.gateway`. Blocks `git pull` and `apt install`.
- **Desktop shows only the wallpaper on a monitor.** gnome-shell runs fine, but the framebuffer is 3840×1080: the forced HDMI-A-1 output plus the real monitor. UI lands on the invisible output. Workaround: use HDMI0. A stray TigerVNC session (`vncserver`, display `:2`) was also found in `~/.xsession-errors`; connect to port 5900, not `:2`.

## Fixed problems

| Problem | Fix | Commit |
|---|---|---|
| Pi froze: point cloud grew without limit and was fully republished every scan | Voxel grid, `voxel_size` param ([ADR 0002](../docs/adr/0002-voxel-grid-instead-of-keyframes.md)) | `c913b7f` |
| Fusion node was one 288-line file | Split into modules ([ADR 0003](../docs/adr/0003-split-fusion-node-into-ros-free-modules.md)); unused `/imu_marker` publisher and dead attributes removed | `6e21694` |
| slam_toolbox YAML without `ros__parameters`, `use_sim_time` at top level | Wrapped correctly | `0429c83` |
| Syntax error in `restrict_scan_by_angle` (`for num_ranges:`) | Fixed loop, `append`, `angle_inc` | `8f0d5a7` |
| Launch file pointed to `mapper_params_online_async.yaml`; `config/` not installed | Corrected name, added `config/*.yaml` to `setup.py` | `8f0d5a7` |
| Submodules without `.gitmodules` (empty folders after clone); MPU9250 link to an unused driver | Added `.gitmodules` ([ADR 0004](../docs/adr/0004-drivers-as-git-submodules.md)), removed MPU9250 link | `4ba5ccf` |
| colcon `build/ install/ log/` committed inside `src/` | Removed | `cea7b63` |
| Suspected wrong gyro unit (deg/s vs rad/s) | Not a bug: driver outputs deg/s, conversion is correct | none |

## Open items

- [ ] Run the full launch on the Pi after all fixes (blocked by internet).
- [ ] Translate points by the device position, not only rotate.
- [ ] Verify/fix the slam_toolbox pose topic and message type.
- [ ] Resolve the circular `odom → base_link` dependency.
- [ ] Decide publishing strategy if the cloud is still slow (timer vs. delta + RViz decay).
- [ ] Finish or remove `restrict_scan_by_angle` (WIP, only computes angles, unused).
- [ ] Add `geometry_msgs`, `tf2_ros`, `sensor_msgs_py` to `package.xml`.
- [ ] Fix `#1/bin/bash` in `watch_ws_lidar.sh` and `Type_simple` in `x11vnc.service`.
- [ ] Add an automated test setup for the ROS-free modules (currently only ad-hoc scripts).
