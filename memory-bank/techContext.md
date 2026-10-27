# Tech Context

Setup and run commands live in the [README](../README.md). This file holds the facts an agent needs to reason about the system.

## Platform

- Raspberry Pi 5, Ubuntu 24.04, GNOME 46, ROS 2 Jazzy. Hostname `robotpi`, user `robotpi`.
- Workspace on the Pi: `~/ws_lidar` (= repo root). A Python venv `ros_venv` is active in the user's shell; its role is not documented.
- Dev machine: Windows without ROS.

## Hardware

| Device | Details |
|---|---|
| RPLidar C1 | USB, `/dev/ttyUSB0`, 460800 baud, `frame_id=laser`. Needs user in `dialout`. |
| MPU6050 | I2C bus 1 (`/dev/i2c-1`), address `0x68`, pins 1/3/5/6. Needs I2C enabled and user in `i2c`. |

## Drivers (git submodules, pinned commits)

- `src/rplidar_ros`: Slamtec, commit `24cc9b6`.
- `src/ros2_mpu6050_driver`: hiwad-aziz, commit `ffddef8`. Publishes `/imu` at 100 Hz, `frame_id=base_link`. **Gyro in deg/s** (verified in `mpu6050sensor.cpp`), accel in m/s². Orientation field is zero. Calibrates on every start (device flat, z up, still).

Decision: [ADR 0004](../docs/adr/0004-drivers-as-git-submodules.md).

## Dependencies

- slam_toolbox from apt (`ros-jazzy-slam-toolbox`), not from source: [ADR 0005](../docs/adr/0005-slam-toolbox-from-apt.md).
- `libi2c-dev` for the IMU driver.
- Python: `rclpy`, `sensor_msgs_py`, `tf2_ros`, `geometry_msgs`. `package.xml` does not declare `geometry_msgs`, `tf2_ros`, `sensor_msgs_py` yet.

## Parameter files

- `config/mapper_params_online.yaml`: slam_toolbox, wrapped in `slam_toolbox: ros__parameters:`. Installed via `setup.py` (`config/*.yaml`).
- `voxel_size` for the fusion node: set in `launch/fusion_launch.py`, default 0.05 m. Read once at start.

## Raspberry Pi system setup

`raspi_config/` mirrors system files (paths relative to `/`). Decision: [ADR 0006](../docs/adr/0006-x11vnc-with-xorg-and-forced-hdmi.md).

- `etc/gdm3/custom.conf`: autologin `robotpi`, `WaylandEnable=false`.
- `etc/systemd/system/x11vnc.service` + `usr/local/bin/x11vnc-auto.sh`: x11vnc on port 5900, attaches to the running Xorg.
- `boot/firmware/cmdline.txt`: `video=HDMI-A-1:1920x1080@60D` forces an output for headless VNC. Consequence: a real monitor must use **HDMI0**.
- `home/robotpi/.vnc/passwd`: placeholder text only, no password.

## Helper scripts

- `build_fusion.sh`: build + source; only works with `source build_fusion.sh`.
- `watch_ws_lidar.sh`: rebuild on change, needs `inotify-tools`. First line has a typo (`#1/bin/bash`).
