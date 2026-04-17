# Architecture

The first-release architecture is intentionally narrow and explicit:

```text
QGroundControl <- UDP 14550 <- PX4 SITL <- TCP 4560 -> Python bridge <-> MuJoCo
                                                             |
                                                             +-> UAV/scene_uav_delta.xml
```

## Responsibilities

- `bridge.py`
  Reads MuJoCo state, converts frames, publishes MAVLink HIL messages, and applies PX4 actuator outputs back into MuJoCo.
- `UAV/`
  Stores the default UAV model, scene files, and mesh assets used by the official workflow.
- `scripts/`
  Owns bootstrap, dependency setup, PX4 patching, PX4 build, validation, and launch orchestration.
- `integrations/px4/`
  Stores the minimal PX4 patch required to run this repository's UAV model with PX4 SITL and QGroundControl.

## First-Release Rules

1. PX4 source code stays external to this repository.
2. QGroundControl stays external to this repository.
3. PX4 is patched minimally and still built as `px4_sitl_default`.
4. The Python bridge is the only official MuJoCo workflow for the first release.
5. ROS 2 is not part of the initial bring-up path yet.

## Why This Shape

The goal is reproducibility, not feature breadth.

By keeping the supported path narrow:

- users do not need to understand multiple simulation backends
- PX4 integration remains small and auditable
- the repository can stay focused on your custom UAV model
- it becomes much easier to test setup on a clean Ubuntu machine
