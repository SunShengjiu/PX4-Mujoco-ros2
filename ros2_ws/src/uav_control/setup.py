from setuptools import find_packages
from setuptools import setup

package_name = "uav_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PX4 MuJoCo ROS2 Maintainers",
    maintainer_email="user@example.com",
    description="Generic UAV control, planning, and test nodes for PX4 Offboard.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hover_test = uav_control.hover_test:main",
            "waypoint_cruise = uav_control.waypoint_cruise:main",
        ],
    },
)
