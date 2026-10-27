# Project Brief

Foundation for all other memory-bank files. Change only when goals or scope change.

## Goal

Scan an entire room in 3D with a handheld device built from a 2D LiDAR and an IMU, and show the resulting point cloud **live** on a monitor connected to the Raspberry Pi.

## Requirements

- Handheld: the user sweeps and tilts the scanner by hand; the third dimension comes from that motion.
- Must also work when the user stands in one place and scans for a long time before moving on.
- Runs entirely on a Raspberry Pi 5 without freezing over a full room scan.
- Live display via RViz on the Pi's monitor (or over VNC).

## Scope

- In scope: sensor fusion, point cloud generation and downsampling, live visualization, Pi setup.
- Out of scope for now: offline post-processing, mesh generation, export formats.

## Context

Private hobby project. Single developer.
