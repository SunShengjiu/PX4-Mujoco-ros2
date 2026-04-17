PYTHON ?= python3
MODEL ?= UAV/scene_uav_delta.xml

.PHONY: bootstrap deps-ubuntu px4-patch px4-build doctor doctor-model run-local run-bridge run-px4 run-qgc run-stack py-compile shell-check

doctor:
	$(PYTHON) scripts/doctor.py --model $(MODEL)

doctor-model:
	$(PYTHON) scripts/doctor.py --model $(MODEL) --model-only

run-local:
	./scripts/run_bridge.sh --no-mavlink

run-bridge:
	./scripts/run_bridge.sh

run-px4:
	./scripts/run_px4.sh

run-qgc:
	./scripts/run_qgc.sh

run-stack:
	./scripts/run_stack.sh

bootstrap:
	./scripts/bootstrap_workspace.sh

deps-ubuntu:
	./scripts/install_ubuntu_deps.sh

px4-patch:
	./scripts/apply_px4_patch.sh

px4-build:
	./scripts/build_px4.sh

py-compile:
	$(PYTHON) -m py_compile bridge.py frames.py scripts/doctor.py

shell-check:
	bash -n scripts/*.sh
