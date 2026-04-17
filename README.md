# PX4 + MuJoCo + QGroundControl Bridge

This repository packages a Python-based MuJoCo bridge, a custom UAV model, and a reproducible PX4 integration workflow so that a fresh Ubuntu machine can be prepared for PX4 + MuJoCo joint simulation with minimal manual patching.

The intended first-class stack is:

- PX4 SITL for flight control
- MuJoCo for physics and visualization
- QGroundControl for the ground station
- This repository's Python bridge for MAVLink HIL between PX4 and MuJoCo
- This repository's UAV model as the default simulated vehicle

## Supported First Release

- Official workflow inputs:
  - this repository
  - a PX4 source tree
  - QGroundControl
- Ubuntu `22.04` or `24.04`
- External PX4 source tree on branch `release/1.15`
- External QGroundControl AppImage
- Default vehicle model: `UAV/scene_uav_delta.xml`
- Default PX4 airframe: `22002_mujoco_delta`
- Default PX4 build target: `px4_sitl_default`

The official simulation path in this repository uses the UAV model shipped here. It does not use PX4's stock x500 or Gazebo models as the primary workflow.

## What This Repository Contains

- A Python MuJoCo bridge: [`bridge.py`](bridge.py)
- The default UAV model and meshes: [`UAV/scene_uav_delta.xml`](UAV/scene_uav_delta.xml)
- PX4 patch automation for a clean external PX4 clone: [`integrations/px4/patches/release-1.15-mujoco-delta.patch`](integrations/px4/patches/release-1.15-mujoco-delta.patch)
- Setup, validation, build, and run scripts under [`scripts/`](scripts)

## Default Data Flow

```text
QGroundControl <- UDP 14550 <- PX4 SITL <- TCP 4560 -> Python Bridge <-> MuJoCo
                                                             |
                                                             +-> UAV/scene_uav_delta.xml
```

## Quick Start

### 1. Prepare the three required pieces

The official workflow is:

1. get the PX4 source tree
2. get QGroundControl
3. get this repository

Clone PX4 separately if you have not already:

```bash
git clone --recursive --branch release/1.15 https://github.com/PX4/PX4-Autopilot.git
```

Download QGroundControl separately and make sure the AppImage is available.

Then clone this repository:

```bash
git clone https://github.com/Zang153/PX4-MuJoCo-ROS2.git
cd PX4-MuJoCo-ROS2
```

Copy the example environment file:

```bash
cp configs/project.env.example configs/project.env
```

Point this repository to your PX4 source tree:

```bash
PX4_MUJOCO_PX4_DIR=/absolute/path/to/PX4-Autopilot
```

If you have a QGroundControl AppImage already downloaded, set:

```bash
PX4_MUJOCO_QGC_APP=/absolute/path/to/QGroundControl.AppImage
```

### 2. Optional: bootstrap a local PX4 clone from this repository

If you already cloned PX4 yourself and set `PX4_MUJOCO_PX4_DIR`, you can skip this step.

```bash
make bootstrap
```

By default this creates:

```text
external/PX4-Autopilot
```

on branch:

```text
release/1.15
```

### 3. Install system and Python dependencies

```bash
make deps-ubuntu
```

This step does three things:

1. installs Ubuntu runtime packages needed by MuJoCo
2. runs PX4's official Ubuntu setup script with `--no-nuttx --no-sim-tools`
3. installs this repository's Python requirements

If you manage Python yourself, make sure these packages are available:

```bash
python3 -m pip install -r requirements.txt
```

### 4. Make sure QGroundControl is executable

If needed:

```bash
chmod +x /path/to/QGroundControl.AppImage
```

Then make sure `PX4_MUJOCO_QGC_APP` in `configs/project.env` points to it.

### 5. Apply the PX4 patch

```bash
make px4-patch
```

The patch only performs the minimum changes required for this repository:

- adds airframe `22002_mujoco_delta`
- registers that airframe in PX4 ROMFS
- configures the PX4 GCS MAVLink link to send to `127.0.0.1:14550`

### 6. Build PX4 SITL

```bash
make px4-build
```

The expected output directory is:

```text
external/PX4-Autopilot/build/px4_sitl_default
```

### 7. Validate the workspace

Full validation:

```bash
make doctor
```

Model-only validation:

```bash
make doctor-model
```

The full check verifies:

- Python modules
- required shell tools
- MuJoCo model contract
- PX4 clone path and branch
- whether the PX4 patch is applied
- whether PX4 SITL build artifacts exist
- whether a QGroundControl AppImage is configured

### 8. Launch the full stack

```bash
make run-stack
```

This launches, in order:

1. QGroundControl, if configured
2. the Python MuJoCo bridge
3. PX4 SITL

If you want to skip QGroundControl:

```bash
./scripts/run_stack.sh --no-qgc
```

## Separate Entry Points

Start only the local MuJoCo bridge without PX4:

```bash
make run-local
```

Start only the bridge with MAVLink enabled:

```bash
make run-bridge
```

Start only PX4:

```bash
make run-px4
```

Start only QGroundControl:

```bash
make run-qgc
```

## Expected Behavior

When `make run-stack` succeeds, you should see:

- a MuJoCo viewer window using `UAV/scene_uav_delta.xml`
- PX4 SITL starting without airframe or MAVLink initialization errors
- QGroundControl connecting automatically on the local machine
- changing attitude, position, and actuator states visible through PX4/QGC

The practical first-release acceptance criteria are:

1. PX4 can arm
2. the vehicle can take off
3. the vehicle can hover for a short period
4. the vehicle can land and disarm again

## The Vehicle Used by This Workflow

The official first-release workflow uses the vehicle model in this repository:

- [`UAV/scene_uav_delta.xml`](UAV/scene_uav_delta.xml)
- [`UAV/UAV_Delta.xml`](UAV/UAV_Delta.xml)

The flight actuators expected by the bridge are:

- `motor_1`
- `motor_2`
- `motor_3`
- `motor_4`

The Quad X motor order is fixed as:

1. front-left
2. rear-right
3. front-right
4. rear-left

The Delta manipulator actuators remain in the model, but PX4 motor outputs are mapped only to the flight motors.

## Important Configuration Keys

Most users only need these:

- `PX4_MUJOCO_PX4_DIR`
- `PX4_MUJOCO_QGC_APP`
- `PX4_MUJOCO_CONDA_ENV`
- `PX4_MUJOCO_MODEL`

The full first-release set in [`configs/project.env.example`](configs/project.env.example) includes:

- `PX4_MUJOCO_PYTHON`
- `PX4_MUJOCO_CONDA_ENV`
- `PX4_MUJOCO_MODEL`
- `PX4_MUJOCO_PX4_DIR`
- `PX4_MUJOCO_PX4_BRANCH`
- `PX4_MUJOCO_PX4_BUILD_DIR`
- `PX4_MUJOCO_PX4_AUTOSTART`
- `PX4_MUJOCO_PX4_SIM_MODEL`
- `PX4_MUJOCO_MAVLINK_HOST`
- `PX4_MUJOCO_TCP_PORT`
- `PX4_MUJOCO_HOVER_THRUST`
- `PX4_MUJOCO_QGC_APP`
- `PX4_MUJOCO_QGC_UDP_PORT`
- `PX4_MUJOCO_ROS2_SETUP`

## Troubleshooting

### `make doctor` reports `pymavlink missing`

Install Python dependencies again:

```bash
python3 -m pip install -r requirements.txt
```

Without `pymavlink`, the bridge can only run in `--no-mavlink` local mode.

### `make px4-patch` fails

Common causes:

1. the PX4 tree is not on `release/1.15`
2. the PX4 working tree is not clean
3. `PX4_MUJOCO_PX4_DIR` points to a different clone than expected

### `make run-stack` does not start QGroundControl

Check:

- `PX4_MUJOCO_QGC_APP` is set
- the AppImage path exists
- the AppImage is executable

### QGroundControl starts but does not connect

Check:

1. the PX4 patch is applied
2. `px4-rc.mavlink` contains `-o 14550 -t 127.0.0.1`
3. ports `14550` and `4560` are not already in use

### PX4 starts but MuJoCo does not react

Check:

1. `make doctor-model` passes
2. `make run-bridge` starts correctly
3. the bridge is listening on TCP port `4560`

## Useful Commands

```bash
make bootstrap
make deps-ubuntu
make px4-patch
make px4-build
make doctor
make run-stack
```

Equivalent direct scripts:

```bash
./scripts/bootstrap_workspace.sh
./scripts/install_ubuntu_deps.sh
./scripts/apply_px4_patch.sh
./scripts/build_px4.sh
./scripts/doctor.py
./scripts/run_stack.sh
```

## Repository Layout

- [`bridge.py`](bridge.py): Python MAVLink HIL bridge
- [`UAV/`](UAV): MuJoCo scenes, meshes, and the default UAV model
- [`scripts/`](scripts): setup, validation, patch, build, and run scripts
- [`integrations/px4/`](integrations/px4): PX4 integration notes and patch assets
- [`docs/`](docs): architecture, status, and model contract notes

## Current Project Status

The repository is already structured for reproducible setup, but flight tuning is still an active task. The current state is:

- the bridge entry points are implemented
- the UAV model contract is in place
- the PX4 patch workflow is automated
- local smoke checks pass
- final hover tuning and full end-to-end validation on a clean machine should still be completed

## Next Direction

After the PX4 + MuJoCo + QGroundControl loop is stable, the next stage is to integrate ROS 2 nodes on top of this stack while keeping PX4 responsible for flight control and MuJoCo responsible for visualization and physics.
