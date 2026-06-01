from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="-1.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("rate_hz", default_value="20.0"),
            Node(
                package="uav_control",
                executable="hover_test",
                name="hover_test",
                output="screen",
                parameters=[
                    {
                        "x": LaunchConfiguration("x"),
                        "y": LaunchConfiguration("y"),
                        "z": LaunchConfiguration("z"),
                        "yaw": LaunchConfiguration("yaw"),
                        "rate_hz": LaunchConfiguration("rate_hz"),
                    }
                ],
            ),
        ]
    )
