# 0005. Install slam_toolbox from apt

- Status: Accepted
- Date: 2026-10-27

## Context

On 2025-08-31 slam_toolbox was built from source inside the workspace. The build failed because CMake could not find `bondcpp`.

## Decision

Install slam_toolbox as a binary package: `sudo apt install ros-jazzy-slam-toolbox`. Do not keep it in `src/`.

## Alternatives considered

- Build from source and install the missing dependencies: slower builds on the Pi, more to maintain, no need for source changes.

## Consequences

- Version follows the Jazzy apt release.
- Its example configs are in `/opt/ros/jazzy/share/slam_toolbox/config/` and serve as the format reference for `mapper_params_online.yaml`.
- Installation needs internet on the Pi.
