from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoints",
                default_value=(
                    "[0.0, 0.0, -1.0, "
                    "1.0, 0.0, -1.0, "
                    "1.0, 1.0, -1.0, "
                    "0.0, 1.0, -1.0]"
                ),
            ),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("rate_hz", default_value="20.0"),
            DeclareLaunchArgument("arrival_radius", default_value="0.25"),
            DeclareLaunchArgument("hold_time_s", default_value="2.0"),
            DeclareLaunchArgument("cmd_pose_topic", default_value="/offboard_control/cmd_pose"),
            DeclareLaunchArgument("odom_topic", default_value="/offboard_control/odom"),
            DeclareLaunchArgument("frame_id", default_value="map_ned"),
            DeclareLaunchArgument("loop", default_value="true"),
            Node(
                package="uav_control",
                executable="waypoint_cruise",
                name="waypoint_cruise",
                output="screen",
                parameters=[
                    {
                        "waypoints": LaunchConfiguration("waypoints"),
                        "yaw": LaunchConfiguration("yaw"),
                        "rate_hz": LaunchConfiguration("rate_hz"),
                        "arrival_radius": LaunchConfiguration("arrival_radius"),
                        "hold_time_s": LaunchConfiguration("hold_time_s"),
                        "cmd_pose_topic": LaunchConfiguration("cmd_pose_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "loop": LaunchConfiguration("loop"),
                    }
                ],
            ),
        ]
    )
