"""Concurrent fleet: peer awareness plus the traffic manager.

peer_obstacles is not optional here. The scan plane sits 0.2 m above base_link
while the tallest part of a robot reaches 0.075 m, so without shared poses the
robots are invisible to each other and drive straight through their neighbours.
Serial running got away with it; concurrent running does not.

    ros2 launch nav2_mobile_robot fleet.launch.py robots:=r1,r2

Expects multi_gazebo.launch.py and multi_nav2.launch.py to already be up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context, *args, **kwargs):
    pkg = get_package_share_directory("nav2_mobile_robot")
    names = [n.strip() for n in LaunchConfiguration("robots").perform(context).split(",") if n.strip()]
    fleet = LaunchConfiguration("fleet").perform(context)

    return [
        Node(
            package="nav2_mobile_robot",
            executable="peer_obstacles",
            name="peer_obstacles",
            output="screen",
            # 0.22 rather than the 0.16 default. A standing robot is not a
            # fleet-level blocker (that is what deadlocked the fleet), so the
            # costmap alone keeps peers apart -- and 0.16 + robot_radius 0.18
            # only demands 0.34 m centre-to-centre, which measured as low as
            # 0.30 m, i.e. touching. 0.22 asks for 0.40 m and leaves margin.
            parameters=[{"use_sim_time": True}, {"robots": names},
                        {"radius": 0.22}],
        ),
        Node(
            package="nav2_mobile_robot",
            executable="fleet_manager",
            name="fleet_manager",
            output="screen",
            # The params file carries platforms and routes; `robots` selects
            # which of those routes to actually run.
            parameters=[fleet, {"use_sim_time": True}, {"robots": names}],
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("nav2_mobile_robot")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robots", default_value="r1,r2"),
            DeclareLaunchArgument(
                "fleet",
                default_value=os.path.join(pkg, "fleet.yaml"),
                description="Platforms, routes and deconfliction tuning.",
            ),
            OpaqueFunction(function=nodes),
        ]
    )
