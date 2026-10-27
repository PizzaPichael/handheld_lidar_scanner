# 0003. Split the fusion node into ROS-free modules

- Status: Accepted
- Date: 2026-10-27

## Context

`fusion_calculation_node.py` was a single 288-line class mixing ROS wiring, quaternion math, filtering, scan projection, point storage and message construction, plus dead code. The dev machine has no ROS, so none of it could be tested there.

## Decision

Split into small modules: `geometry`, `orientation_filter`, `scan_projection`, `point_cloud_accumulator` (all ROS-free), `ros_messages` (message builders) and a thin `fusion_calculation_node` that only wires them to ROS. Topic names, frames, node name and entry point stay unchanged.

## Alternatives considered

- Keep one file and only tidy it: still untestable without ROS and hard to read.

## Consequences

- Logic can be tested with plain Python on the dev machine. The refactor was verified bit-identical against the old code this way.
- The node uses relative imports, so it only starts via `ros2 run` / `ros2 launch`, not `python3 fusion_calculation_node.py`.
- New logic should go into a ROS-free module, not into the node.
