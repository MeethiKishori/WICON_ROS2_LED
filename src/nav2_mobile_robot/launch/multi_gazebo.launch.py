"""Spawn N namespaced robots into one Gazebo world.

Each robot rX gets:
  - its own URDF, with every link/joint/TF frame prefixed  rX/base_link, ...
  - its own Gazebo topics                                  /rX/cmd_vel, /rX/lidar, ...
  - its own robot_state_publisher in namespace /rX, TF on  /rX/tf
  - its own ros_gz_bridge

The /clock bridge is launched once, globally. Without it every node started with
use_sim_time:=true sits at t=0 forever and timer callbacks never fire.
"""

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

# Validated free spawn poses for worlds/small_world.sdf (2x2 m, boxes at
# (0.4,0.3) and (-0.4,-0.3)). Each has >0.27 m clearance to any obstacle -- the
# nav footprint is 0.18 m -- and the poses are >=1.4 m apart. Yaw faces the centre.
SPAWN_POSES = [
    (-0.70, 0.70, -0.785),
    (0.70, -0.70, 2.356),
    (0.70, 0.70, -2.356),
    (-0.70, -0.70, 0.785),
]

# Every link and joint is already prefixed (r1/base_link, r2/base_link ...), so
# all robots can share the global /tf without colliding. That lets ONE RViz show
# the whole fleet, instead of one window per robot subscribed to its own /rX/tf.
TF_REMAP = []


def robot_actions(context, *args, **kwargs):
    pkg = get_package_share_directory("nav2_mobile_robot")
    xacro_file = os.path.join(pkg, "nav2_mobile_robot.xacro")

    names = [n.strip() for n in LaunchConfiguration("robots").perform(context).split(",") if n.strip()]
    samples = LaunchConfiguration("lidar_samples").perform(context)

    if len(names) > len(SPAWN_POSES):
        raise RuntimeError(
            f"{len(names)} robots requested but only {len(SPAWN_POSES)} validated "
            f"spawn poses exist for this 2x2 m world. Add more to SPAWN_POSES."
        )

    actions = []
    for i, name in enumerate(names):
        x, y, yaw = SPAWN_POSES[i]
        description = Command(
            ["xacro ", xacro_file, f" prefix:={name}/ ns:=/{name} lidar_samples:={samples}"]
        )

        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=name,
                output="screen",
                parameters=[{"robot_description": description, "use_sim_time": True}],
                remappings=TF_REMAP,
            )
        )

        actions.append(
            Node(
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
                # The odometry plugin publishes its TF on a side topic; land it
                # on the global /tf like every other TF source.
                remappings=[(f"/{name}/odom/tf", "/tf")],
            )
        )

        # Stagger the spawns. Creating several models in the same physics step
        # makes Gazebo drop entities and occasionally explodes the solver.
        actions.append(
            TimerAction(
                period=3.0 + 2.0 * i,
                actions=[
                    Node(
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
                ],
            )
        )

    return actions


def generate_launch_description():
    pkg = get_package_share_directory("nav2_mobile_robot")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments=[
            ("gz_args", [" -r -v 4 ", LaunchConfiguration("headless"), " ", LaunchConfiguration("world")])
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robots",
                default_value="r1,r2",
                description="Comma-separated robot names, one namespace each (e.g. r1,r2,r3)",
            ),
            DeclareLaunchArgument("world", default_value="small_world.sdf"),
            DeclareLaunchArgument(
                "headless",
                default_value="",
                description='Pass "-s" to run Gazebo server-only, no GUI.',
            ),
            DeclareLaunchArgument(
                "lidar_samples",
                default_value="720",
                description="Lidar rays per scan. 3600 is the single-robot value; "
                "lowered here because GPU lidar cost multiplies with robot count.",
            ),
            SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=pkg),
            gazebo,
            clock_bridge,
            OpaqueFunction(function=robot_actions),
        ]
    )
