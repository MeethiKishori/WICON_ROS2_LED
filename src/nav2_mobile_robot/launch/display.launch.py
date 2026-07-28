from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock from Gazebo. Must be true when running alongside gazebo.launch.py",
    )

    standalone = LaunchConfiguration("standalone")
    declare_standalone = DeclareLaunchArgument(
        "standalone",
        default_value="true",
        description=(
            "True to view the URDF on its own. Set false alongside gazebo.launch.py, "
            "which already publishes robot and joint states"
        ),
    )

    xacro_path = "nav2_mobile_robot.xacro"

    robot_description = PathJoinSubstitution(
        [get_package_share_directory("nav2_mobile_robot"), xacro_path]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(standalone),
        parameters=[
            {
                "robot_description": Command(["xacro ", robot_description]),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(standalone),
        parameters=[{"use_sim_time": use_sim_time}],
    )
    rviz_file = "rviz.rviz"
    rviz_path = PathJoinSubstitution(
        [get_package_share_directory("nav2_mobile_robot"), rviz_file]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_standalone,
            robot_state_publisher_node,
            joint_state_publisher_gui,
            rviz2_node,
        ]
    )
