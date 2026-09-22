"""One RViz for the whole fleet, with a goal tool you can click for each robot.

All robots share the global /tf because their frames are prefixed
(r1/base_link, r2/base_link ...), so a single window can show every robot. The
per-robot displays -- RobotModel, LaserScan, Path -- are cloned out of
rviz/rviz.rviz once per robot and pointed at that robot's topics; Grid and Map
stay shared.

Pair it with fleet.launch.py, which drives both robots through their
waypoints:

    ros2 launch nav2_mobile_robot multi_rviz.launch.py robots:=r1,r2
    ros2 launch nav2_mobile_robot fleet.launch.py robots:=r1,r2
"""

import copy
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Displays that exist once for the fleet rather than once per robot.
SHARED_DISPLAYS = {"rviz_default_plugins/Grid", "rviz_default_plugins/Map"}
# Distinct colours so you can tell the robots apart at a glance (r, g, b).
COLOURS = ["255; 85; 0", "0; 170; 255", "0; 255; 127", "255; 0; 212"]


def _namespace_topics(node, ns):
    """Point every absolute topic in a display at one robot's namespace."""
    if isinstance(node, dict):
        return {
            key: (
                f"/{ns}{value}"
                if key == "Value" and isinstance(value, str) and value.startswith("/")
                else _namespace_topics(value, ns)
            )
            for key, value in node.items()
        }
    if isinstance(node, list):
        return [_namespace_topics(v, ns) for v in node]
    return node


def _config_for(names, src, outdir):
    with open(src) as fh:
        cfg = yaml.safe_load(fh)

    manager = cfg["Visualization Manager"]
    original = manager["Displays"]

    displays = [d for d in original if d.get("Class") in SHARED_DISPLAYS]
    for index, name in enumerate(names):
        for template in original:
            if template.get("Class") in SHARED_DISPLAYS:
                continue
            display = _namespace_topics(copy.deepcopy(template), name)
            display["Name"] = f"{name} {template.get('Name', '')}".strip()
            if display.get("Class") == "rviz_default_plugins/Path":
                display["Color"] = COLOURS[index % len(COLOURS)]
            displays.append(display)

    manager["Displays"] = displays
    # The one frame every robot shares.
    manager["Global Options"]["Fixed Frame"] = "map"

    path = os.path.join(outdir, "rviz_fleet.rviz")
    with open(path, "w") as fh:
        yaml.safe_dump(cfg, fh, default_flow_style=False)
    return path


def rviz_node(context, *args, **kwargs):
    pkg = get_package_share_directory("nav2_mobile_robot")
    names = [n.strip() for n in LaunchConfiguration("robots").perform(context).split(",") if n.strip()]
    config = _config_for(names, os.path.join(pkg, "rviz.rviz"), tempfile.mkdtemp(prefix="rviz_fleet_"))

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=["-d", config],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robots", default_value="r1,r2"),
            OpaqueFunction(function=rviz_node),
        ]
    )
