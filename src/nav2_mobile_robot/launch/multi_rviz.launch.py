import copy
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "nav2_mobile_robot"

SHARED_DISPLAYS = {"rviz_default_plugins/Grid", "rviz_default_plugins/Map"}
COLOURS = ["255; 85; 0", "0; 170; 255", "0; 255; 127", "255; 0; 212"]


def read_robot_names(context):
    robots_text = LaunchConfiguration("robots").perform(context)

    robot_names = []
    for piece in robots_text.split(","):
        name = piece.strip()
        if name != "":
            robot_names.append(name)
    return robot_names


def add_namespace(display, ns):
    if isinstance(display, dict):
        result = {}
        for key, value in display.items():
            is_topic = key == "Value" and isinstance(value, str) and value.startswith("/")
            if is_topic:
                result[key] = f"/{ns}{value}"
            else:
                result[key] = add_namespace(value, ns)
        return result

    if isinstance(display, list):
        result = []
        for item in display:
            result.append(add_namespace(item, ns))
        return result

    return display


def is_shared(display):
    return display.get("Class") in SHARED_DISPLAYS


def write_fleet_config(robot_names, source_file, output_folder):
    with open(source_file) as f:
        config = yaml.safe_load(f)

    manager = config["Visualization Manager"]
    original_displays = manager["Displays"]

    displays = []
    for display in original_displays:
        if is_shared(display):
            displays.append(display)

    for index, name in enumerate(robot_names):
        for template in original_displays:
            if is_shared(template):
                continue

            display = add_namespace(copy.deepcopy(template), name)
            display["Name"] = f"{name} {template.get('Name', '')}".strip()

            if display.get("Class") == "rviz_default_plugins/Path":
                display["Color"] = COLOURS[index % len(COLOURS)]

            displays.append(display)

    manager["Displays"] = displays
    manager["Global Options"]["Fixed Frame"] = "map"

    output_file = os.path.join(output_folder, "rviz_fleet.rviz")
    with open(output_file, "w") as f:
        yaml.safe_dump(config, f, default_flow_style=False)
    return output_file


def rviz_node(context, *args, **kwargs):
    package_folder = get_package_share_directory(PACKAGE_NAME)
    source_file = os.path.join(package_folder, "rviz.rviz")
    output_folder = tempfile.mkdtemp(prefix="rviz_fleet_")

    robot_names = read_robot_names(context)
    config_file = write_fleet_config(robot_names, source_file, output_folder)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", config_file],
    )
    return [rviz]


def generate_launch_description():
    robots_argument = DeclareLaunchArgument("robots", default_value="r1,r2")
    start_rviz = OpaqueFunction(function=rviz_node)

    return LaunchDescription([
        robots_argument,
        start_rviz,
    ])
