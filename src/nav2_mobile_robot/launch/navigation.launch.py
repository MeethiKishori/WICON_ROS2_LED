# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, PushROSNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


PACKAGE_NAME = "nav2_mobile_robot"

NAV2_SERVERS = [
    {
        "package": "nav2_controller",
        "name": "controller_server",
        "plugin": "nav2_controller::ControllerServer",
        "sends_cmd_vel": True,
    },
    {
        "package": "nav2_smoother",
        "name": "smoother_server",
        "plugin": "nav2_smoother::SmootherServer",
        "sends_cmd_vel": False,
    },
    {
        "package": "nav2_planner",
        "name": "planner_server",
        "plugin": "nav2_planner::PlannerServer",
        "sends_cmd_vel": False,
    },
    {
        "package": "nav2_behaviors",
        "name": "behavior_server",
        "plugin": "behavior_server::BehaviorServer",
        "sends_cmd_vel": True,
    },
    {
        "package": "nav2_bt_navigator",
        "name": "bt_navigator",
        "plugin": "nav2_bt_navigator::BtNavigator",
        "sends_cmd_vel": False,
    },
    {
        "package": "nav2_waypoint_follower",
        "name": "waypoint_follower",
        "plugin": "nav2_waypoint_follower::WaypointFollower",
        "sends_cmd_vel": False,
    },
    {
        "package": "nav2_velocity_smoother",
        "name": "velocity_smoother",
        "plugin": "nav2_velocity_smoother::VelocitySmoother",
        "sends_cmd_vel": True,
    },
    {
        "package": "nav2_collision_monitor",
        "name": "collision_monitor",
        "plugin": "nav2_collision_monitor::CollisionMonitor",
        "sends_cmd_vel": False,
    },
    {
        "package": "opennav_docking",
        "executable": "opennav_docking",
        "name": "docking_server",
        "plugin": "opennav_docking::DockingServer",
        "sends_cmd_vel": False,
    },
]

LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "collision_monitor",
    "bt_navigator",
    "waypoint_follower",
]


def remappings_for(server):
    if server["sends_cmd_vel"]:
        return [("cmd_vel", "cmd_vel_nav")]
    return []


def make_server_node(server, params, use_respawn, log_level):
    executable = server.get("executable", server["name"])
    return Node(
        package=server["package"],
        executable=executable,
        name=server["name"],
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings_for(server),
    )


def make_composable_server(server, params):
    return ComposableNode(
        package=server["package"],
        plugin=server["plugin"],
        name=server["name"],
        parameters=[params],
        remappings=remappings_for(server),
    )


def generate_launch_description():
    package_folder = get_package_share_directory(PACKAGE_NAME)

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    container_full_name = (namespace, "/", container_name)

    params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    separate_nodes = [
        PushROSNamespace(namespace),
        SetParameter("use_sim_time", use_sim_time),
    ]
    for server in NAV2_SERVERS:
        separate_nodes.append(make_server_node(server, params, use_respawn, log_level))
    separate_nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{"autostart": autostart}, {"node_names": LIFECYCLE_NODES}],
        )
    )

    composable_nodes = []
    for server in NAV2_SERVERS:
        composable_nodes.append(make_composable_server(server, params))
    composable_nodes.append(
        ComposableNode(
            package="nav2_lifecycle_manager",
            plugin="nav2_lifecycle_manager::LifecycleManager",
            name="lifecycle_manager_navigation",
            parameters=[{"autostart": autostart, "node_names": LIFECYCLE_NODES}],
        )
    )

    run_as_separate_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=separate_nodes,
    )

    run_in_container = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            PushROSNamespace(namespace),
            SetParameter("use_sim_time", use_sim_time),
            LoadComposableNodes(
                target_container=container_full_name,
                composable_node_descriptions=composable_nodes,
            ),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level namespace",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([package_folder, "nav.yaml"]),
            description="Full path to the ROS2 parameters file to use for all launched nodes",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the nav2 stack",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="False",
            description="Use composed bringup if True",
        ),
        DeclareLaunchArgument(
            "container_name",
            default_value="nav2_container",
            description="The name of the container that nodes load into if using composition",
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Whether to respawn if a node crashes. Applied when composition is disabled.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Log level",
        ),
        run_as_separate_nodes,
        run_in_container,
    ])
