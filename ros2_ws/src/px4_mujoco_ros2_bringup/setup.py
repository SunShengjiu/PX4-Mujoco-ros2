from setuptools import setup

package_name = "px4_mujoco_ros2_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/offboard_control.launch.py",
                "launch/hover_test.launch.py",
                "launch/offboard_hover_test.launch.py",
                "launch/waypoint_cruise.launch.py",
                "launch/offboard_waypoint_cruise.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PX4 MuJoCo ROS2 Maintainers",
    maintainer_email="user@example.com",
    description="Launch files for the PX4 MuJoCo ROS 2 stack.",
    license="MIT",
)
