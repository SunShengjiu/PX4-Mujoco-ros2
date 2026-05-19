# Local Visual Hover

## Purpose

Use this path when you want to see the aircraft move in MuJoCo immediately, without waiting for PX4, ROS 2, or QGroundControl.

## Command

```bash
make run-local
```

## What Runs

- MuJoCo
- `bridge.py`
- the bridge-local takeoff/hover controller

PX4 is not started in this path.

## Expected Result

- the MuJoCo window opens
- the vehicle rises from the ground
- the vehicle settles near the configured target height

## Notes

- this is the fastest visual validation path in the repository
- this path is useful for confirming the MuJoCo scene and flight actuation behavior
- this path does not validate the PX4 message chain
