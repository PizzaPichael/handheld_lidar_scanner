# AGENTS.md

Handheld 3D room scanner: a 2D LiDAR (RPLidar C1) and an IMU (MPU6050) on a Raspberry Pi 5 with ROS 2 Jazzy. Each 2D scan is rotated by the IMU orientation, so sweeping the device by hand builds a 3D point cloud. Human-facing docs are in [README.md](README.md).

## Start of every task

1. Read **all** files in [memory-bank/](memory-bank/), starting with `projectbrief.md`. They are the project's memory between sessions.
2. Check [docs/adr/](docs/adr/) before changing anything an ADR covers. Do not reverse an accepted decision without writing a new ADR that supersedes it.

## End of every task

- Update `memory-bank/activeContext.md` (focus, recent changes, next steps) and `memory-bank/progress.md` (problems found, fixed, open).
- Made a design decision with a real trade-off? Add an ADR (see [docs/adr/README.md](docs/adr/README.md)).
- Changed architecture, hardware, setup or build steps? Update `systemPatterns.md` / `techContext.md` and the README.
- Only record what is verified. Mark assumptions and untested changes as such.

## Environments

- **Dev machine:** Windows, **no ROS installed**. Pure-logic modules can be tested with plain Python. ROS wiring cannot.
- **Raspberry Pi** (`robotpi`, workspace `~/ws_lidar` = repo root): runs ROS. All real testing happens there, done by the user.

## Build and run (on the Pi)

```bash
cd ~/ws_lidar
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch imu_lidar_sensor_fusion fusion_launch.py use_rviz:=true
```

## Conventions

- Own code lives only in `src/imu_lidar_sensor_fusion/`. `src/rplidar_ros` and `src/ros2_mpu6050_driver` are submodules: do not edit them.
- Keep logic in small ROS-free modules and keep `fusion_calculation_node.py` limited to ROS wiring (see [ADR 0003](docs/adr/0003-split-fusion-node-into-ros-free-modules.md)).
- Identifiers in English, code comments in German, documentation in English.
- Commit messages: conventional style (`feat:`, `fix:`, `refactor:`, `docs:`, `chore:`). The user commits; do not commit or push unless asked.
- Do not commit colcon output (`build/`, `install/`, `log/`).
