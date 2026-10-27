# Active Context

Last updated: 2026-10-27

## Current focus

Getting the whole pipeline running on the Pi again after a long break, with a point cloud that no longer freezes the Pi.

## Recent changes (2026-10-27)

- Repo cleanup: colcon output removed from `src/`, `.gitmodules` added, unused MPU9250 submodule link removed.
- Fixed: syntax error in the fusion node, slam_toolbox config filename, config not installed, YAML missing `ros__parameters`.
- README added.
- Fusion node split into ROS-free modules (behaviour verified identical off-device).
- Voxel grid downsampling with parameter `voxel_size` (logic tested off-device).
- Agent docs added: this memory bank, `docs/adr/`, `AGENTS.md`.
- Scan points are now translated by the device position after rotation (tested off-device).
- Slam pose subscription fixed: `/pose` as `PoseWithCovarianceStamped` (verified in slam_toolbox source, not yet on the Pi).
- Branch `raspi_remote` (one commit, `25f432e`) is **abandoned**; work continues on `main`. The Pi may still have it checked out.

## Next steps

1. Pi: restore internet (see Raspberry Pi section in `progress.md`), `git checkout main && git pull`.
2. Pi: `colcon build`, launch, confirm everything starts. Check `ros2 param get /slam_toolbox max_laser_range` = `12.0` and `ros2 param get /fusion_node voxel_size`.
3. Confirm `/pose` arrives (`ros2 topic echo /pose`) and the cloud follows the device when walking.
4. Then tackle the circular TF.

## Open decisions

- Publishing strategy if the cloud is still too slow: timer at 1–2 Hz vs. publish only new points with RViz decay time.
