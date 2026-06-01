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
            DeclareLaunchArgument("warmup_setpoints", default_value="20"),
            DeclareLaunchArgument("command_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("auto_request_offboard_and_arm", default_value="true"),
            DeclareLaunchArgument("auto_request_offboard", default_value="true"),
            DeclareLaunchArgument("auto_arm", default_value="true"),
            DeclareLaunchArgument("require_local_position_for_offboard", default_value="true"),
            DeclareLaunchArgument("px4_input_prefix", default_value="/fmu/in"),
            DeclareLaunchArgument("px4_output_prefix", default_value="/fmu/out"),
            Node(
                package="px4_mujoco_ros2_control",
                executable="offboard_control",
                name="offboard_control",
                output="screen",
                parameters=[
                    {
                        "rate_hz": LaunchConfiguration("rate_hz"),
                        "warmup_setpoints": LaunchConfiguration("warmup_setpoints"),
                        "command_timeout_s": LaunchConfiguration("command_timeout_s"),
                        "auto_request_offboard_and_arm": LaunchConfiguration(
                            "auto_request_offboard_and_arm"
                        ),
                        "auto_request_offboard": LaunchConfiguration("auto_request_offboard"),
                        "auto_arm": LaunchConfiguration("auto_arm"),
                        "require_local_position_for_offboard": LaunchConfiguration(
                            "require_local_position_for_offboard"
                        ),
                        "px4_input_prefix": LaunchConfiguration("px4_input_prefix"),
                        "px4_output_prefix": LaunchConfiguration("px4_output_prefix"),
                    }
                ],
            ),
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
