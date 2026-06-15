from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
import os

#Adaptive Monte Carlo Localization

def generate_launch_description():

    amcl_yaml = os.path.join(
        get_package_share_directory("nav2_mobile_robot"), "amcl.yaml"
    )
    map_file = os.path.join(
        get_package_share_directory("nav2_mobile_robot"), "map.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[amcl_yaml],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"use_sim_time": True}, {"yaml_filename": map_file}],
                emulate_tty=True,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {"use_sim_time": True},
                    {"autostart": True},
                    {"node_names": ["map_server", "amcl"]},
                ],
            ),
        ]
    )
