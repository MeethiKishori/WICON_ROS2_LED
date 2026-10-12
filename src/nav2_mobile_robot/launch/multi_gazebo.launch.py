import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = "nav2_mobile_robot"

SPAWN_POSES = [
    (-0.70, 0.70, -0.785),
    (0.70, -0.70, 2.356),
    (0.70, 0.70, -2.356),
    (-0.70, -0.70, 0.785),
]


def read_robot_names(context):
    robots_text = LaunchConfiguration("robots").perform(context)

    robot_names = []
    for piece in robots_text.split(","):
        name = piece.strip()
        if name != "":
            robot_names.append(name)
    return robot_names


def make_state_publisher(name, robot_description):
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=name,
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )


def make_bridge(name):
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge",
        namespace=name,
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            f"/{name}/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            f"/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/{name}/odom/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            f"/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[(f"/{name}/odom/tf", "/tf")],
    )


def make_spawner(name, x, y, yaw, delay):
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{name}",
        output="screen",
        arguments=[
            "-name", name,
            "-topic", f"/{name}/robot_description",
            "-x", str(x),
            "-y", str(y),
            "-z", "0.1",
            "-Y", str(yaw),
        ],
    )
    return TimerAction(period=delay, actions=[spawn_node])


def robot_actions(context, *args, **kwargs):
    package_folder = get_package_share_directory(PACKAGE_NAME)
    xacro_file = os.path.join(package_folder, "nav2_mobile_robot.xacro")

    robot_names = read_robot_names(context)
    lidar_samples = LaunchConfiguration("lidar_samples").perform(context)

    if len(robot_names) > len(SPAWN_POSES):
        raise RuntimeError(
            f"{len(robot_names)} robots requested but only {len(SPAWN_POSES)} validated "
            f"spawn poses exist for this 2x2 m world. Add more to SPAWN_POSES."
        )

    actions = []
    for i, name in enumerate(robot_names):
        x, y, yaw = SPAWN_POSES[i]

        xacro_args = f" prefix:={name}/ ns:=/{name} lidar_samples:={lidar_samples}"
        robot_description = Command(["xacro ", xacro_file, xacro_args])

        spawn_delay = 3.0 + 2.0 * i

        actions.append(make_state_publisher(name, robot_description))
        actions.append(make_bridge(name))
        actions.append(make_spawner(name, x, y, yaw, spawn_delay))

    return actions


def generate_launch_description():
    package_folder = get_package_share_directory(PACKAGE_NAME)

    gz_sim_launch_file = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    gz_args = [" -r -v 4 ", LaunchConfiguration("headless"), " ", LaunchConfiguration("world")]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch_file]),
        launch_arguments=[("gz_args", gz_args)],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    robots_argument = DeclareLaunchArgument(
        "robots",
        default_value="r1,r2",
        description="Comma-separated robot names, one namespace each (e.g. r1,r2,r3)",
    )

    world_argument = DeclareLaunchArgument("world", default_value="small_world.sdf")

    headless_argument = DeclareLaunchArgument(
        "headless",
        default_value="",
        description='Pass "-s" to run Gazebo server-only, no GUI.',
    )

    lidar_samples_argument = DeclareLaunchArgument(
        "lidar_samples",
        default_value="720",
        description="Lidar rays per scan. 3600 is the single-robot value; "
        "lowered here because GPU lidar cost multiplies with robot count.",
    )

    resource_path = SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=package_folder)

    start_robots = OpaqueFunction(function=robot_actions)

    return LaunchDescription([
        robots_argument,
        world_argument,
        headless_argument,
        lidar_samples_argument,
        resource_path,
        gazebo,
        clock_bridge,
        start_robots,
    ])
