PYTHON ?= python3
MODEL ?= UAV/scene_uav_delta.xml

.PHONY: bootstrap deps-ubuntu px4-patch px4-build ros2-build doctor doctor-model run-local run-bridge run-px4 stop-px4 run-qgc run-stack run-stack-uav run-stack-delta run-ros2-agent run-offboard-control py-compile shell-check

doctor:
	$(PYTHON) scripts/doctor.py --model $(MODEL)

doctor-model:
	$(PYTHON) scripts/doctor.py --model $(MODEL) --model-only

run-local:
	./scripts/run_bridge.sh --no-mavlink --local-hover

run-bridge:
	./scripts/run_bridge.sh

run-px4:
	./scripts/run_px4.sh

stop-px4:
	./scripts/stop_px4.sh

run-qgc:
	./scripts/run_qgc.sh

run-stack:
	./scripts/run_stack.sh

run-stack-uav:
	./scripts/run_stack_uav.sh

run-stack-delta:
	./scripts/run_stack_delta.sh

bootstrap:
	./scripts/bootstrap_workspace.sh

deps-ubuntu:
	./scripts/install_ubuntu_deps.sh

px4-patch:
	./scripts/apply_px4_patch.sh

px4-build:
	./scripts/build_px4.sh

ros2-build:
	./scripts/build_ros2.sh

run-ros2-agent:
	./scripts/run_ros2_agent.sh

run-offboard-control:
	./scripts/run_offboard_control.sh

py-compile:
	$(PYTHON) -m py_compile bridge.py frames.py scripts/doctor.py ros2_ws/src/px4_mujoco_ros2_control/px4_mujoco_ros2_control/*.py

shell-check:
	bash -n scripts/*.sh
