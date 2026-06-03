from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("center_x", default_value="0.0"),
            DeclareLaunchArgument("center_y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="-1.0"),
            DeclareLaunchArgument("radius", default_value="0.8"),
            DeclareLaunchArgument("period_s", default_value="24.0"),
            DeclareLaunchArgument("pre_circle_hold_s", default_value="5.0"),
            DeclareLaunchArgument("hold_time_s", default_value="999999.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("rate_hz", default_value="20.0"),
            DeclareLaunchArgument("clockwise", default_value="false"),
            DeclareLaunchArgument("use_current_position", default_value="true"),
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
                executable="waypoint_cruise",
                name="waypoint_cruise",
                output="screen",
                parameters=[
                    {
                        "center_x": LaunchConfiguration("center_x"),
                        "center_y": LaunchConfiguration("center_y"),
                        "z": LaunchConfiguration("z"),
                        "radius": LaunchConfiguration("radius"),
                        "period_s": LaunchConfiguration("period_s"),
                        "pre_circle_hold_s": LaunchConfiguration("pre_circle_hold_s"),
                        "hold_time_s": LaunchConfiguration("hold_time_s"),
                        "yaw": LaunchConfiguration("yaw"),
                        "rate_hz": LaunchConfiguration("rate_hz"),
                        "clockwise": LaunchConfiguration("clockwise"),
                        "use_current_position": LaunchConfiguration("use_current_position"),
                    }
                ],
            ),
        ]
    )
