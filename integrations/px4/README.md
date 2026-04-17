# PX4 Integration

The first-release workflow does not vendor PX4 source code into this repository and does not require users to manually edit PX4 files.

This directory stores the minimum PX4-side integration assets needed for the supported workflow.

## Included Asset

- `patches/release-1.15-mujoco-delta.patch`

This patch is intended for a clean external PX4 clone on branch `release/1.15`.

## What The Patch Changes

1. Adds the custom airframe `22002_mujoco_delta`
2. Registers that airframe in `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`
3. Updates `px4-rc.mavlink` so PX4 sends the GCS MAVLink link to `127.0.0.1:14550`

## Why The First Release Uses A Minimal Patch

The supported workflow is:

- this repository owns the Python MuJoCo bridge
- the external PX4 clone remains a standard `px4_sitl_default` build
- this repository owns launch orchestration for bridge, PX4, and QGroundControl

This keeps the integration easier to reason about and easier to validate on a fresh Ubuntu machine.
