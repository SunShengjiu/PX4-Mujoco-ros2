from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="-2.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("rate_hz", default_value="20.0"),
            DeclareLaunchArgument("warmup_setpoints", default_value="20"),
            DeclareLaunchArgument("command_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("auto_request_offboard_and_arm", default_value="true"),
            DeclareLaunchArgument("use_estimator_flags_gate", default_value="false"),
            DeclareLaunchArgument("require_ev_vel", default_value="false"),
            Node(
                package="px4_mujoco_ros2_control",
                executable="offboard_control",
                name="offboard_control",
                output="screen",
                parameters=[
                    {
                        "x": LaunchConfiguration("x"),
                        "y": LaunchConfiguration("y"),
                        "z": LaunchConfiguration("z"),
                        "yaw": LaunchConfiguration("yaw"),
                        "rate_hz": LaunchConfiguration("rate_hz"),
                        "warmup_setpoints": LaunchConfiguration("warmup_setpoints"),
                        "command_timeout_s": LaunchConfiguration("command_timeout_s"),
                        "auto_request_offboard_and_arm": LaunchConfiguration("auto_request_offboard_and_arm"),
                        "use_estimator_flags_gate": LaunchConfiguration("use_estimator_flags_gate"),
                        "require_ev_vel": LaunchConfiguration("require_ev_vel"),
                    }
                ],
            ),
        ]
    )
