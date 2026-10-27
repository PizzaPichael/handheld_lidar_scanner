# Handheld LiDAR Scanner

A private hobby project: a handheld 3D room scanner built from a **2D LiDAR** and an **IMU** on a Raspberry Pi 5, running ROS 2.

The LiDAR measures a flat 2D slice of its surroundings. The IMU tracks how the device is tilted and turned. By rotating each 2D scan with the current IMU orientation, sweeping the scanner by hand builds up a 3D point cloud. The goal is to scan a whole room and show the result live on a screen attached to the Pi.

## Hardware

| Component | Model | Connection |
|---|---|---|
| Computer | Raspberry Pi 5 | |
| LiDAR | Slamtec RPLidar C1 | USB (`/dev/ttyUSB0`, 460800 baud) |
| IMU | MPU6050 | I2C bus 1, address `0x68` |

### Wiring (MPU6050)

| MPU6050 | Raspberry Pi pin |
|---|---|
| VCC | Pin 1 (3.3 V) |
| GND | Pin 6 (GND) |
| SDA | Pin 3 (GPIO 2) |
| SCL | Pin 5 (GPIO 3) |

I2C must be enabled on the Pi, and the user needs access to the devices:

```bash
sudo usermod -aG dialout,i2c $USER   # serial port (LiDAR) and I2C (IMU), re-login afterwards
```

## Software

- Ubuntu 24.04
- ROS 2 Jazzy
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) (from apt)
- Drivers as git submodules: [rplidar_ros](https://github.com/Slamtec/rplidar_ros), [ros2_mpu6050_driver](https://github.com/hiwad-aziz/ros2_mpu6050_driver)

## Repository layout

The repository root is the colcon workspace (on the Pi: `~/ws_lidar`).

```
.
├── src/
│   ├── imu_lidar_sensor_fusion/   # own package: fusion node, launch file, configs
│   ├── rplidar_ros/               # submodule: LiDAR driver
│   └── ros2_mpu6050_driver/       # submodule: IMU driver
├── raspi_config/                  # Pi system files for headless VNC access
├── build_fusion.sh                # build + source in one step
└── watch_ws_lidar.sh              # rebuild automatically on file changes
```

### Package `imu_lidar_sensor_fusion`

| Path | Purpose |
|---|---|
| `imu_lidar_sensor_fusion/fusion_calculation_node.py` | ROS node: subscribers, publishers, TF, wires the modules below together |
| `imu_lidar_sensor_fusion/orientation_filter.py` | `ComplementaryFilter`: orientation from gyro and accelerometer |
| `imu_lidar_sensor_fusion/scan_projection.py` | Converts a `LaserScan` into rotated 3D points |
| `imu_lidar_sensor_fusion/point_cloud_accumulator.py` | Collects the points of all scans with voxel grid downsampling |
| `imu_lidar_sensor_fusion/geometry.py` | Quaternion math |
| `imu_lidar_sensor_fusion/ros_messages.py` | Builds Imu, Pose, TF and PointCloud2 messages |
| `launch/fusion_launch.py` | Starts drivers, fusion node, slam_toolbox and optionally RViz |
| `config/mapper_params_online.yaml` | slam_toolbox parameters |
| `rviz2/fusion_config.rviz` | RViz layout |
| `test_scripts/lidar_test.py` | Standalone node that prints scan parameters |

## Setup

```bash
git clone --recurse-submodules https://github.com/PizzaPichael/handheld_lidar_scanner.git ~/ws_lidar
# already cloned without submodules:
git submodule update --init

sudo apt install ros-jazzy-slam-toolbox libi2c-dev
```

## Build and run

```bash
cd ~/ws_lidar
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

ros2 launch imu_lidar_sensor_fusion fusion_launch.py use_rviz:=true
```

Both `source` commands apply to the current terminal only. Add them to `~/.bashrc` to make them permanent.

`build_fusion.sh` runs the build and sources the result, but only takes effect when sourced itself (`source build_fusion.sh`, not `./build_fusion.sh`). `watch_ws_lidar.sh` rebuilds on every change in `src/` and needs `inotify-tools`.

**IMU calibration:** the MPU6050 driver calibrates on every start. Keep the scanner flat (z-axis up) and still until the node reports it is done.

Leave out `use_rviz:=true` when running over SSH without a display.

## Data flow

```
RPLidar C1 ──/scan──────────┬──────────────► slam_toolbox ──/map, TF map→odom
                            │                     │
                            ▼                     │ /pose
MPU6050 ──/imu──► fusion_calculation_node ◄───────┘
                     │
                     ├── /imu_fused    (Imu, with orientation)
                     ├── /imu_pose     (PoseStamped)
                     ├── /pointcloud   (PointCloud2, voxel-downsampled)
                     └── TF odom→base_link
```

**fusion_calculation_node**

- Estimates roll and pitch with a complementary filter (gyro integration corrected by the accelerometer, `alpha = 0.9`). Yaw comes from gyro integration only. The MPU6050 driver reports angular velocity in deg/s, the node converts it to rad/s.
- Projects each scan into XY points, rotates them by the latest IMU orientation, translates them by the position from slam_toolbox and adds them to the point cloud, which is published on every scan.
- The point cloud is voxel-downsampled: space is divided into cubes of edge length `voxel_size` (parameter, default `0.05` m) and only the first point per cube is kept. The cloud grows with the scanned area, not with scan time. Change it in `launch/fusion_launch.py` (`parameters=[{'voxel_size': 0.05}]` on the fusion node, then rebuild) or, when running the node on its own, with `--ros-args -p voxel_size:=0.03`. Check the active value with `ros2 param get /fusion_node voxel_size`.
- Broadcasts `odom → base_link` using the IMU orientation and the position from slam_toolbox.

slam_toolbox publishes `/pose` (`PoseWithCovarianceStamped`, frame `map`) only after it has successfully processed a scan. Until then the position stays at the origin.

**TF tree**

| Transform | Published by |
|---|---|
| `map → odom` | slam_toolbox |
| `odom → base_link` | fusion node (dynamic) |
| `base_link → laser` | fusion node (static, identity) |

The RViz fixed frame is `odom`.

## Raspberry Pi setup (headless VNC)

`raspi_config/` mirrors system files on the Pi. Each file belongs at the same path relative to `/`:

| File | Purpose |
|---|---|
| `etc/gdm3/custom.conf` | Automatic login for `robotpi`, Wayland disabled (x11vnc needs Xorg) |
| `etc/systemd/system/x11vnc.service` | Starts x11vnc at boot |
| `usr/local/bin/x11vnc-auto.sh` | Waits for the running Xorg, finds its display and auth file, attaches x11vnc on port 5900 |
| `home/robotpi/.vnc/passwd` | Placeholder only, the real file is created on the Pi with `x11vnc -storepasswd` |
| `boot/firmware/cmdline.txt` | Adds `video=HDMI-A-1:1920x1080@60D`, forcing a 1920x1080 output so VNC has a resolution without a monitor (`cmdline.txt.bak` is the original) |

Install:

```bash
sudo apt install x11vnc
sudo cp raspi_config/etc/gdm3/custom.conf /etc/gdm3/
sudo cp raspi_config/etc/systemd/system/x11vnc.service /etc/systemd/system/
sudo cp raspi_config/usr/local/bin/x11vnc-auto.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/x11vnc-auto.sh
sudo systemctl enable --now x11vnc
x11vnc -storepasswd                  # creates ~/.vnc/passwd (run without sudo)
```

Connect with any VNC client to `<pi-address>:5900`.

**Monitor port:** because HDMI-A-1 is forced on, plug a real monitor into **HDMI0** (the port next to the USB-C power input). On the other port GNOME sees two screens and may put the panel, dock and desktop icons on the invisible one, leaving only the wallpaper visible.
