"""Per-robot Nav2 stacks for the namespaced robots spawned by multi_gazebo.launch.py.

Localization is a static identity map -> rX/odom transform, not AMCL or SLAM:
gz-sim-odometry-publisher-system reports pose in *world* coordinates (verified:
a robot spawned at (-0.70, 0.70) publishes exactly that on /rX/odom), and
small_world.yaml places the map at the world origin. So odom already is the map
frame and there is nothing to correct. That keeps the fit experiment free of
localization error.

nav.yaml is the single source of truth. It is written for one robot, with absolute
topics (/lidar, /odom) and bare frames (base_link, odom), so it is rewritten per
robot here rather than duplicated into a second file that would drift.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Frame parameters whose value is a bare frame name needing the robot prefix.
FRAME_KEYS = {"robot_base_frame", "base_frame_id", "odom_frame_id", "local_frame"}
# Frames that stay shared across the whole fleet.
SHARED_FRAMES = {"map"}
# Topics that stay shared across the whole fleet.
SHARED_TOPICS = {"/map", "/clock"}


def _rewrite(node, ns):
    """Namespace absolute topics and bare frame names in a nav2 param tree.

    Relative topic values ("cmd_vel", "local_costmap/costmap_raw") are left alone:
    they already resolve inside the node's own namespace.
    """
    if isinstance(node, dict):
        out = {}
        for key, value in node.items():
            if isinstance(value, str):
                if key in FRAME_KEYS and value not in SHARED_FRAMES:
                    value = f"{ns}/{value}"
                elif key == "global_frame" and value == "odom":
                    value = f"{ns}/odom"
                elif (
                    key.endswith("topic") or key.endswith("_topic")
                ) and value.startswith("/") and value not in SHARED_TOPICS:
                    value = f"/{ns}{value}"
            else:
                value = _rewrite(value, ns)
            out[key] = value
        return out
    if isinstance(node, list):
        return [_rewrite(v, ns) for v in node]
    return node


def _params_for(ns, src, outdir):
    with open(src) as fh:
        params = yaml.safe_load(fh)

    params = _rewrite(params, ns)

    # The static layer must read the one shared map, not a namespace-relative
    # topic that would resolve to /<ns>/<costmap>/map and never arrive.
    for costmap in ("local_costmap", "global_costmap"):
        layers = params.get(costmap, {}).get(costmap, {}).get("ros__parameters", {})
        if "static_layer" in layers:
            layers["static_layer"]["map_topic"] = "/map"

    path = os.path.join(outdir, f"nav_{ns}.yaml")
    with open(path, "w") as fh:
        yaml.safe_dump(params, fh, default_flow_style=False)
    return path


def robot_actions(context, *args, **kwargs):
    pkg = get_package_share_directory("nav2_mobile_robot")
    names = [n.strip() for n in LaunchConfiguration("robots").perform(context).split(",") if n.strip()]
    outdir = tempfile.mkdtemp(prefix="nav2_multi_")

    actions = []
    for name in names:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                namespace=name,
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=["--frame-id", "map", "--child-frame-id", f"{name}/odom"],
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg, "navigation.launch.py")),
                launch_arguments={
                    "namespace": name,
                    "use_sim_time": "true",
                    "params_file": _params_for(name, os.path.join(pkg, "nav.yaml"), outdir),
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    pkg = get_package_share_directory("nav2_mobile_robot")

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

    return LaunchDescription(
        [
            DeclareLaunchArgument("robots", default_value="r1,r2"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(pkg, "small_world.yaml"),
                description="Shared static map. small_world.yaml is generated from the\n"
                "world geometry, so it reaches the walls exactly.",
            ),
            map_server,
            map_lifecycle,
            OpaqueFunction(function=robot_actions),
        ]
    )
