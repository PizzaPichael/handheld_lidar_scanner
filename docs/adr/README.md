# Architecture Decision Records

One file per decision: `NNNN-short-title.md`, numbered sequentially. Each record must make sense on its own.

## Rules

- Write an ADR when a decision has a real trade-off or would look "wrong" to someone without the context.
- Never edit an accepted ADR's decision. To change it, write a new ADR and set the old one to `Superseded by NNNN`.
- Status: `Proposed`, `Accepted`, `Rejected`, `Superseded by NNNN`.
- Only record reasons that were actually given. Do not invent a rationale after the fact.

## Index

| # | Title | Status |
|---|---|---|
| [0001](0001-record-architecture-decisions.md) | Record architecture decisions | Accepted |
| [0002](0002-voxel-grid-instead-of-keyframes.md) | Voxel grid instead of keyframes for point cloud downsampling | Accepted |
| [0003](0003-split-fusion-node-into-ros-free-modules.md) | Split the fusion node into ROS-free modules | Accepted |
| [0004](0004-drivers-as-git-submodules.md) | Include drivers as git submodules | Accepted |
| [0005](0005-slam-toolbox-from-apt.md) | Install slam_toolbox from apt | Accepted |
| [0006](0006-x11vnc-with-xorg-and-forced-hdmi.md) | x11vnc with Xorg and a forced HDMI output for headless access | Accepted |

## Template

```markdown
# NNNN. Title

- Status: Proposed
- Date: YYYY-MM-DD

## Context
What problem or force makes a decision necessary?

## Decision
What we do.

## Alternatives considered
What else was possible and why it lost.

## Consequences
What becomes easier, what becomes harder, what to watch out for.
```
