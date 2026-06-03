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
            DeclareLaunchArgument("cmd_pose_topic", default_value="/offboard_control/cmd_pose"),
            DeclareLaunchArgument("odom_topic", default_value="/offboard_control/odom"),
            DeclareLaunchArgument("frame_id", default_value="map_ned"),
            DeclareLaunchArgument("clockwise", default_value="false"),
            DeclareLaunchArgument("use_current_position", default_value="true"),
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
                        "cmd_pose_topic": LaunchConfiguration("cmd_pose_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "clockwise": LaunchConfiguration("clockwise"),
                        "use_current_position": LaunchConfiguration("use_current_position"),
                    }
                ],
            ),
        ]
    )
