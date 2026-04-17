#!/usr/bin/env python3

from __future__ import annotations

import argparse
import importlib.util
import os
import platform
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_FILE = REPO_ROOT / "configs" / "project.env"
DEFAULTS = {
    "PX4_MUJOCO_PYTHON": "python3",
    "PX4_MUJOCO_CONDA_ENV": "px4-mujoco",
    "PX4_MUJOCO_MODEL": str(REPO_ROOT / "UAV" / "scene_uav_delta.xml"),
    "PX4_MUJOCO_PX4_DIR": "external/PX4-Autopilot",
    "PX4_MUJOCO_PX4_BRANCH": "release/1.15",
    "PX4_MUJOCO_PX4_BUILD_DIR": "build/px4_sitl_default",
    "PX4_MUJOCO_PX4_AUTOSTART": "22002",
    "PX4_MUJOCO_PX4_SIM_MODEL": "mujoco_delta",
    "PX4_MUJOCO_MAVLINK_HOST": "0.0.0.0",
    "PX4_MUJOCO_TCP_PORT": "4560",
    "PX4_MUJOCO_HOVER_THRUST": "0.60",
    "PX4_MUJOCO_QGC_APP": "",
    "PX4_MUJOCO_QGC_UDP_PORT": "14550",
    "PX4_MUJOCO_ROS2_SETUP": "",
}
DEFAULT_MODEL = Path(DEFAULTS["PX4_MUJOCO_MODEL"]).expanduser()
REQUIRED_SENSORS = ("body_gyro", "body_linacc")
OPTIONAL_SENSORS = ("body_mag",)
REQUIRED_FLIGHT_ACTUATORS = ("motor_1", "motor_2", "motor_3", "motor_4")
PYTHON_MODULES = ("numpy", "mujoco", "pymavlink")
REQUIRED_TOOLS = ("bash", "git", "make", "python3")
OPTIONAL_TOOLS = ("cmake", "ninja")


@dataclass
class ModelInspection:
    files: set[Path]
    sensor_names: set[str]
    actuator_count: int
    actuator_names: set[str]
    freejoint_count: int


@dataclass
class WorkspaceConfig:
    model: Path
    px4_dir: Path
    px4_build_dir: Path
    qgc_app: Path | None
    ros2_setup: Path | None
    px4_branch: str
    px4_autostart: str
    px4_sim_model: str
    tcp_port: str
    hover_thrust: str


def load_env_file(path: Path) -> None:
    if not path.exists():
        return

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip())


def config_value(name: str) -> str:
    return os.environ.get(name, DEFAULTS[name])


def resolve_repo_path(value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = (REPO_ROOT / path).resolve()
    return path


def module_installed(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def inspect_model(entry_path: Path) -> ModelInspection:
    visited: set[Path] = set()
    sensor_names: set[str] = set()
    actuator_count = 0
    actuator_names: set[str] = set()
    freejoint_count = 0

    def walk(xml_path: Path) -> None:
        nonlocal actuator_count, freejoint_count

        xml_path = xml_path.resolve()
        if xml_path in visited:
            return
        visited.add(xml_path)

        root = ET.parse(xml_path).getroot()
        sensor_names.update(
            element.attrib["name"]
            for element in root.findall(".//sensor/*")
            if "name" in element.attrib
        )
        actuator_names.update(
            element.attrib["name"]
            for element in root.findall(".//actuator/*")
            if "name" in element.attrib
        )
        actuator_count += len(root.findall(".//actuator/*"))
        freejoint_count += len(root.findall(".//freejoint"))

        for include in root.findall(".//include"):
            include_file = include.attrib.get("file")
            if not include_file:
                continue
            include_path = (xml_path.parent / include_file).resolve()
            if include_path.exists():
                walk(include_path)

    walk(entry_path)
    return ModelInspection(
        files=visited,
        sensor_names=sensor_names,
        actuator_count=actuator_count,
        actuator_names=actuator_names,
        freejoint_count=freejoint_count,
    )


def git_output(repo: Path, *args: str) -> str:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo), *args],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return ""
    return result.stdout.strip()


def git_repo_is_clean(repo: Path) -> bool:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo), "status", "--porcelain"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return False
    return result.stdout.strip() == ""


def file_contains(path: Path, needle: str) -> bool:
    if not path.exists():
        return False
    return needle in path.read_text(encoding="utf-8", errors="ignore")


def print_kv(label: str, value: str) -> None:
    print(f"{label:<24} {value}")


def build_workspace_config(model_override: Path | None) -> WorkspaceConfig:
    model_raw = str(model_override) if model_override is not None else config_value("PX4_MUJOCO_MODEL")
    model_path = Path(model_raw).expanduser()
    if not model_path.is_absolute():
        model_path = (REPO_ROOT / model_path).resolve()

    px4_dir = resolve_repo_path(config_value("PX4_MUJOCO_PX4_DIR"))
    px4_build_raw = config_value("PX4_MUJOCO_PX4_BUILD_DIR")
    px4_build_dir = Path(px4_build_raw).expanduser()
    if not px4_build_dir.is_absolute():
        px4_build_dir = (px4_dir / px4_build_dir).resolve()

    qgc_raw = config_value("PX4_MUJOCO_QGC_APP")
    qgc_path = resolve_repo_path(qgc_raw) if qgc_raw else None

    ros2_raw = config_value("PX4_MUJOCO_ROS2_SETUP")
    ros2_path = resolve_repo_path(ros2_raw) if ros2_raw else None

    return WorkspaceConfig(
        model=model_path,
        px4_dir=px4_dir,
        px4_build_dir=px4_build_dir,
        qgc_app=qgc_path,
        ros2_setup=ros2_path,
        px4_branch=config_value("PX4_MUJOCO_PX4_BRANCH"),
        px4_autostart=config_value("PX4_MUJOCO_PX4_AUTOSTART"),
        px4_sim_model=config_value("PX4_MUJOCO_PX4_SIM_MODEL"),
        tcp_port=config_value("PX4_MUJOCO_TCP_PORT"),
        hover_thrust=config_value("PX4_MUJOCO_HOVER_THRUST"),
    )


def main() -> int:
    load_env_file(CONFIG_FILE)

    parser = argparse.ArgumentParser(description="Check PX4 + MuJoCo + QGC workspace readiness")
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--model-only", action="store_true")
    args = parser.parse_args()

    config = build_workspace_config(args.model)
    failures: list[str] = []
    warnings: list[str] = []

    print("== Workspace ==")
    print_kv("repo", str(REPO_ROOT))
    print_kv("platform", f"{platform.system()} {platform.release()}")
    print_kv("python", sys.version.split()[0])
    print_kv("config", str(CONFIG_FILE) if CONFIG_FILE.exists() else "missing")
    print()

    print("== Python Modules ==")
    for module_name in PYTHON_MODULES:
        installed = module_installed(module_name)
        print_kv(module_name, "ok" if installed else "missing")
        if module_name in {"numpy", "mujoco"} and not installed:
            failures.append(f"missing python module: {module_name}")
        if module_name == "pymavlink" and not installed:
            warnings.append("pymavlink missing, bridge can only run in --no-mavlink mode")
    print()

    print("== Host Tools ==")
    for tool_name in REQUIRED_TOOLS + OPTIONAL_TOOLS:
        present = shutil.which(tool_name) is not None
        print_kv(tool_name, "ok" if present else "missing")
        if tool_name in REQUIRED_TOOLS and not present:
            failures.append(f"missing host tool: {tool_name}")
    print()

    print("== Runtime Config ==")
    print_kv("model", str(config.model))
    print_kv("px4 dir", str(config.px4_dir))
    print_kv("px4 build dir", str(config.px4_build_dir))
    print_kv("px4 branch", config.px4_branch)
    print_kv("autostart", config.px4_autostart)
    print_kv("sim model", config.px4_sim_model)
    print_kv("tcp port", config.tcp_port)
    print_kv("hover thrust", config.hover_thrust)
    print_kv("qgc app", str(config.qgc_app) if config.qgc_app else "unset")
    print_kv("ros2 setup", str(config.ros2_setup) if config.ros2_setup else "unset")
    print()

    print("== Model Contract ==")
    print_kv("entry", str(config.model))
    if not config.model.exists():
        failures.append(f"model file not found: {config.model}")
        print_kv("status", "missing")
    else:
        inspection = inspect_model(config.model)
        missing_required = sorted(set(REQUIRED_SENSORS) - inspection.sensor_names)
        missing_optional = sorted(set(OPTIONAL_SENSORS) - inspection.sensor_names)
        missing_flight_actuators = sorted(set(REQUIRED_FLIGHT_ACTUATORS) - inspection.actuator_names)

        print_kv("xml files", str(len(inspection.files)))
        print_kv("freejoints", str(inspection.freejoint_count))
        print_kv("actuators", str(inspection.actuator_count))
        print_kv(
            "named actuators",
            ", ".join(sorted(inspection.actuator_names)) if inspection.actuator_names else "none",
        )
        print_kv(
            "named sensors",
            ", ".join(sorted(inspection.sensor_names)) if inspection.sensor_names else "none",
        )

        if inspection.freejoint_count == 0:
            failures.append("model has no freejoint; vehicle root pose is not available")
        if inspection.actuator_count == 0:
            failures.append("model has no actuators; PX4 controls cannot be applied")
        if missing_flight_actuators:
            failures.append(
                "model missing required flight actuators: " + ", ".join(missing_flight_actuators)
            )
        if missing_required:
            failures.append(
                "model missing required HIL sensors: " + ", ".join(missing_required)
            )
        if missing_optional:
            warnings.append("model missing optional sensors: " + ", ".join(missing_optional))

        print_kv(
            "flight actuators",
            "ok" if not missing_flight_actuators else "missing: " + ", ".join(missing_flight_actuators),
        )
        print_kv(
            "required sensors",
            "ok" if not missing_required else "missing: " + ", ".join(missing_required),
        )
        print_kv(
            "optional sensors",
            "ok" if not missing_optional else "missing: " + ", ".join(missing_optional),
        )
    print()

    if not args.model_only:
        px4_airframe_file = config.px4_dir / "ROMFS" / "px4fmu_common" / "init.d-posix" / "airframes" / f"{config.px4_autostart}_mujoco_delta"
        px4_airframe_list = config.px4_dir / "ROMFS" / "px4fmu_common" / "init.d-posix" / "airframes" / "CMakeLists.txt"
        px4_gcs_file = config.px4_dir / "ROMFS" / "px4fmu_common" / "init.d-posix" / "px4-rc.mavlink"
        px4_bin = config.px4_build_dir / "bin" / "px4"
        px4_etc = config.px4_build_dir / "etc"
        px4_rootfs = config.px4_build_dir / "rootfs"

        print("== PX4 Workspace ==")
        repo_exists = (config.px4_dir / ".git").exists()
        print_kv("repo", "ok" if repo_exists else "missing")
        if not repo_exists:
            failures.append(f"PX4 repository not found: {config.px4_dir}")
        else:
            current_branch = git_output(config.px4_dir, "branch", "--show-current")
            print_kv("current branch", current_branch or "unknown")
            if current_branch and current_branch != config.px4_branch:
                failures.append(
                    f"PX4 branch mismatch: expected {config.px4_branch}, got {current_branch}"
                )

            clean = git_repo_is_clean(config.px4_dir)
            print_kv("working tree", "clean" if clean else "dirty")
            if not clean:
                warnings.append("PX4 working tree is dirty; patching may fail")

            airframe_exists = px4_airframe_file.exists()
            airframe_registered = file_contains(px4_airframe_list, "22002_mujoco_delta")
            qgc_route = file_contains(px4_gcs_file, "-o 14550 -t 127.0.0.1")
            patch_applied = airframe_exists and airframe_registered and qgc_route

            print_kv("airframe file", "ok" if airframe_exists else "missing")
            print_kv("airframe list", "ok" if airframe_registered else "missing entry")
            print_kv("qgc mavlink route", "ok" if qgc_route else "missing patch")
            print_kv("patch status", "applied" if patch_applied else "missing")

            if not patch_applied:
                failures.append("PX4 patch is not applied")

            print_kv("px4 bin", "ok" if px4_bin.exists() else "missing")
            print_kv("etc dir", "ok" if px4_etc.exists() else "missing")
            print_kv("rootfs dir", "ok" if px4_rootfs.exists() else "missing")
            if not px4_bin.exists() or not px4_etc.exists() or not px4_rootfs.exists():
                failures.append(f"PX4 SITL build artifacts missing under {config.px4_build_dir}")
        print()

        print("== QGroundControl ==")
        if config.qgc_app is None:
            print_kv("appimage", "unset")
            warnings.append("QGroundControl AppImage is unset; run-stack will skip QGC")
        else:
            print_kv("appimage", f"{config.qgc_app} ({'ok' if config.qgc_app.exists() else 'missing'})")
            if not config.qgc_app.exists():
                failures.append(f"QGroundControl AppImage not found: {config.qgc_app}")
        print()

    print("== Summary ==")
    if failures:
        for item in failures:
            print(f"FAIL: {item}")
    else:
        print("FAIL: none")

    if warnings:
        for item in warnings:
            print(f"WARN: {item}")
    else:
        print("WARN: none")

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
