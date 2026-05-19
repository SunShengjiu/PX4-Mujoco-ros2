from setuptools import setup

package_name = "px4_mujoco_ros2_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/offboard_hold.launch.py",
                "launch/offboard_control.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PX4 MuJoCo ROS2 Maintainers",
    maintainer_email="user@example.com",
    description="ROS 2 offboard control helpers for the PX4 MuJoCo simulation stack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "offboard_control = px4_mujoco_ros2_control.offboard_control:main",
            "offboard_hold = px4_mujoco_ros2_control.offboard_hold:main",
        ],
    },
)
