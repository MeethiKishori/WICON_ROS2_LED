import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "nav2_mobile_robot"


def nodes(context, *args, **kwargs):
    robots_text = LaunchConfiguration("robots").perform(context)

    robot_names = []
    for piece in robots_text.split(","):
        name = piece.strip()
        if name != "":
            robot_names.append(name)

    fleet_yaml_path = LaunchConfiguration("fleet").perform(context)

    peer_obstacles_node = Node(
        package=PACKAGE_NAME,
        executable="peer_obstacles",
        name="peer_obstacles",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robots": robot_names},
            {"radius": 0.22},
        ],
    )

    fleet_manager_node = Node(
        package=PACKAGE_NAME,
        executable="fleet_manager",
        name="fleet_manager",
        output="screen",
        parameters=[
            fleet_yaml_path,
            {"use_sim_time": True},
            {"robots": robot_names},
        ],
    )

    return [peer_obstacles_node, fleet_manager_node]


def generate_launch_description():
    package_folder = get_package_share_directory(PACKAGE_NAME)
    default_fleet_yaml = os.path.join(package_folder, "fleet.yaml")

    robots_argument = DeclareLaunchArgument(
        "robots",
        default_value="r1,r2",
        description="Comma-separated robot names.",
    )

    fleet_argument = DeclareLaunchArgument(
        "fleet",
        default_value=default_fleet_yaml,
        description="Platforms, routes and deconfliction tuning.",
    )

    start_nodes = OpaqueFunction(function=nodes)

    return LaunchDescription([
        robots_argument,
        fleet_argument,
        start_nodes,
    ])
