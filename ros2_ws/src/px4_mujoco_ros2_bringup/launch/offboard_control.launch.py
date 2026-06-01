from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rate_hz", default_value="20.0"),
            DeclareLaunchArgument("warmup_setpoints", default_value="20"),
            DeclareLaunchArgument("command_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("auto_request_offboard_and_arm", default_value="true"),
            DeclareLaunchArgument("auto_request_offboard", default_value="true"),
            DeclareLaunchArgument("auto_arm", default_value="true"),
            DeclareLaunchArgument("require_local_position_for_offboard", default_value="true"),
            DeclareLaunchArgument("px4_input_prefix", default_value="/fmu/in"),
            DeclareLaunchArgument("px4_output_prefix", default_value="/fmu/out"),
            DeclareLaunchArgument("trajectory_setpoint_topic", default_value="~/trajectory_setpoint"),
            DeclareLaunchArgument("cmd_pose_topic", default_value="~/cmd_pose"),
            DeclareLaunchArgument("cmd_twist_topic", default_value="~/cmd_twist"),
            DeclareLaunchArgument("vehicle_command_topic", default_value="~/vehicle_command"),
            DeclareLaunchArgument("odom_topic", default_value="~/odom"),
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
                        "trajectory_setpoint_topic": LaunchConfiguration(
                            "trajectory_setpoint_topic"
                        ),
                        "cmd_pose_topic": LaunchConfiguration("cmd_pose_topic"),
                        "cmd_twist_topic": LaunchConfiguration("cmd_twist_topic"),
                        "vehicle_command_topic": LaunchConfiguration("vehicle_command_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                    }
                ],
            ),
        ]
    )
