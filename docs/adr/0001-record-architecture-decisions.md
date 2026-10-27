# 0001. Record architecture decisions

- Status: Accepted
- Date: 2026-10-27

## Context

The project was paused for about a year. On return, the reasons behind earlier choices were gone. AI coding agents also work on the code and tend to "fix" choices they see as suboptimal unless the rationale is written down.

## Decision

Record decisions with a real trade-off as ADRs in `docs/adr/`. Keep the current project state separately in `memory-bank/` (Cline memory bank pattern). `AGENTS.md` tells agents to read both.

## Alternatives considered

- Rationale only in commit messages: hard to find, not read by agents at session start.
- Decisions inside the memory bank: mixes changing state with stable history.

## Consequences

- Every task that makes a decision needs an extra file.
- Older choices (ROS 2, complementary filter, RPLidar C1, MPU6050) have no ADR because their rationale is unknown.
