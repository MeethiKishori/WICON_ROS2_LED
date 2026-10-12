import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "nav2_mobile_robot"

FRAME_KEYS = {"robot_base_frame", "base_frame_id", "odom_frame_id", "local_frame"}
SHARED_FRAMES = {"map"}
SHARED_TOPICS = {"/map", "/clock"}


def read_robot_names(context):
    robots_text = LaunchConfiguration("robots").perform(context)

    robot_names = []
    for piece in robots_text.split(","):
        name = piece.strip()
        if name != "":
            robot_names.append(name)
    return robot_names


def namespace_one_value(key, value, ns):
    if key in FRAME_KEYS and value not in SHARED_FRAMES:
        return f"{ns}/{value}"

    if key == "global_frame" and value == "odom":
        return f"{ns}/odom"

    is_topic = key.endswith("topic")
    is_absolute = value.startswith("/")
    if is_topic and is_absolute and value not in SHARED_TOPICS:
        return f"/{ns}{value}"

    return value


def add_namespace(params, ns):
    if isinstance(params, dict):
        result = {}
        for key, value in params.items():
            if isinstance(value, str):
                result[key] = namespace_one_value(key, value, ns)
            else:
                result[key] = add_namespace(value, ns)
        return result

    if isinstance(params, list):
        result = []
        for item in params:
            result.append(add_namespace(item, ns))
        return result

    return params


def write_params_for_robot(ns, source_file, output_folder):
    with open(source_file) as f:
        params = yaml.safe_load(f)

    params = add_namespace(params, ns)

    for costmap in ("local_costmap", "global_costmap"):
        costmap_params = params.get(costmap, {}).get(costmap, {}).get("ros__parameters", {})
        if "static_layer" in costmap_params:
            costmap_params["static_layer"]["map_topic"] = "/map"

    output_file = os.path.join(output_folder, f"nav_{ns}.yaml")
    with open(output_file, "w") as f:
        yaml.safe_dump(params, f, default_flow_style=False)
    return output_file


def robot_actions(context, *args, **kwargs):
    package_folder = get_package_share_directory(PACKAGE_NAME)
    nav_yaml = os.path.join(package_folder, "nav.yaml")
    navigation_launch_file = os.path.join(package_folder, "navigation.launch.py")
    output_folder = tempfile.mkdtemp(prefix="nav2_multi_")

    actions = []
    for name in read_robot_names(context):
        map_to_odom = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            namespace=name,
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=["--frame-id", "map", "--child-frame-id", f"{name}/odom"],
        )

        robot_params_file = write_params_for_robot(name, nav_yaml, output_folder)

        nav2_stack = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_file),
            launch_arguments={
                "namespace": name,
                "use_sim_time": "true",
                "params_file": robot_params_file,
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        )

        actions.append(map_to_odom)
        actions.append(nav2_stack)

    return actions


def generate_launch_description():
    package_folder = get_package_share_directory(PACKAGE_NAME)

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"yaml_filename": LaunchConfiguration("map")},
            {"frame_id": "map"},
        ],
    )

    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    robots_argument = DeclareLaunchArgument("robots", default_value="r1,r2")

    map_argument = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(package_folder, "small_world.yaml"),
        description="Shared static map. small_world.yaml is generated from the\n"
        "world geometry, so it reaches the walls exactly.",
    )

    start_robots = OpaqueFunction(function=robot_actions)

    return LaunchDescription([
        robots_argument,
        map_argument,
        map_server,
        map_lifecycle,
        start_robots,
    ])
