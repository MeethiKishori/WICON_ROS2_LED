from setuptools import find_packages, setup

package_name = "nav2_mobile_robot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name,
            [
                "launch/multi_gazebo.launch.py",
                "launch/multi_nav2.launch.py",
                "launch/multi_rviz.launch.py",
                "launch/fleet.launch.py",
                "launch/navigation.launch.py",
                "urdf/nav2_mobile_robot.xacro",
                "urdf/nav2_mobile_robot_macro.xacro",
                "urdf/gazebo_control.xacro",
                "urdf/lidar.xacro",
                "urdf/odom_plugin.xacro",
                "rviz/rviz.rviz",
                "worlds/small_world.sdf",
                "config/nav.yaml",
                "config/fleet.yaml",
                "map/small_world.pgm",
                "map/small_world.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="blackbeard",
    maintainer_email="abubakarmughal92@gmail.com",
    description="Two namespaced robots running waypoint routes concurrently under Nav2",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fleet_manager = nav2_mobile_robot.fleet_manager:main",
            "peer_obstacles = nav2_mobile_robot.peer_obstacles:main",
        ],
    },
)
