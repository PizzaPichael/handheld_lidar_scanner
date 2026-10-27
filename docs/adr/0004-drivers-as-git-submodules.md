# 0004. Include drivers as git submodules

- Status: Accepted
- Date: 2026-10-27

## Context

The LiDAR and IMU drivers were cloned into `src/` on the Pi and accidentally committed as bare gitlinks without a `.gitmodules` file. Other clones got empty folders and `git submodule update` could not fetch anything.

## Decision

Keep the drivers as proper submodules with a `.gitmodules` file, pinned to the commits that run on the Pi:

- `src/rplidar_ros` → `https://github.com/Slamtec/rplidar_ros.git` @ `24cc9b6`
- `src/ros2_mpu6050_driver` → `https://github.com/hiwad-aziz/ros2_mpu6050_driver.git` @ `ffddef8`

The link to `ros2_mpu9250_driver` was removed: it was not on the Pi and the launch file uses the MPU6050.

## Alternatives considered

- Vendor the driver code into the repo: bigger repo, harder to pull upstream fixes.
- Remove the links and document manual clones: no pinned versions.

## Consequences

- Clone with `--recurse-submodules` or run `git submodule update --init`.
- Driver code must not be edited in place. Local changes would need a fork.
