"""Run odometry, with optional RViz and bag-compatible simulated time."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("st3821671")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "odom.yaml")),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("points_topic", default_value="/points"),
        DeclareLaunchArgument("save_trajectory", default_value="true"),
        DeclareLaunchArgument("trajectory_path", default_value="./estimate.txt"),
        Node(
            package="st3821671", executable="odom_node", name="lidar_odometry",
            output="screen",
            parameters=[LaunchConfiguration("params_file"), {
                "use_sim_time": use_sim_time,
                "save_trajectory": ParameterValue(LaunchConfiguration("save_trajectory"), value_type=bool),
                "trajectory_path": ParameterValue(LaunchConfiguration("trajectory_path"), value_type=str),
            }],
            # Remap the conventional input without overriding a topic set in YAML.
            remappings=[("/points", LaunchConfiguration("points_topic"))],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", os.path.join(share, "rviz", "odom.rviz")],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[("/points", LaunchConfiguration("points_topic"))],
        ),
    ])
