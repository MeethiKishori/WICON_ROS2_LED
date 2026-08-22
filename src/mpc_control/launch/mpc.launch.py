"""Run the acados MPC in place of nav2's controller_server.

Starts only what the MPC needs:
  planner_server   -- global paths (nav2, configured from nav.yaml)
  lifecycle_manager -- brings planner_server up
  goal_to_plan     -- RViz goal  -> /plan
  mpc_control      -- /plan + /odom -> /cmd_vel

Deliberately NOT started: controller_server, velocity_smoother,
collision_monitor, behavior_server. They all write to the /cmd_vel chain and
would fight the MPC for the robot.

Gazebo, SLAM and RViz are launched separately -- see the README.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    default_params = os.path.join(
        get_package_share_directory('nav2_mobile_robot'), 'nav.yaml')

    # Defaults to false: gazebo.launch.py's ros_gz_bridge does NOT bridge
    # /clock, so with use_sim_time:=true the ROS clock stays at 0, timers
    # never fire and the controller publishes nothing. Set this to true only
    # after adding a /clock entry to that bridge.
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use the Gazebo clock (requires /clock to be bridged)')

    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='nav2 parameter file (planner_server + global_costmap)')

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mpc',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['planner_server'],
        }],
    )

    goal_to_plan = Node(
        package='mpc_control',
        executable='goal_to_plan',
        name='goal_to_plan',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    mpc_control = Node(
        package='mpc_control',
        executable='mpc_control',
        name='mpc_control',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(planner_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(goal_to_plan)
    ld.add_action(mpc_control)
    return ld
