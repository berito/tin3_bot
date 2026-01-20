# Main launch file for tin3_bot Gazebo simulation
# Starts Gazebo world and spawns robot

import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_tin3_bot = get_package_share_directory("tin3_bot")

    # Set Gazebo resource path for meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            ':',
            os.path.dirname(pkg_tin3_bot)
        ]
    )

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value="empty.sdf",
        description="Path to the Gazebo world file",
    )

    robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )
    robot_x = DeclareLaunchArgument(
        "robot_x",
        default_value="0.0",
        description="Robot spawn X",
    )
    robot_y = DeclareLaunchArgument(
        "robot_y",
        default_value="0.0",
        description="Robot spawn Y",
    )
    robot_z = DeclareLaunchArgument(
        "robot_z",
        default_value="0.5",
        description="Robot spawn Z",
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
    )

    # Spawn robot
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tin3_bot, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={
            "robot_ns": LaunchConfiguration("robot_ns"),
            "x": LaunchConfiguration("robot_x"),
            "y": LaunchConfiguration("robot_y"),
            "z": LaunchConfiguration("robot_z"),
        }.items(),
    )

    # Bridge clock topic
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_resource_path,
            sim_world,
            robot_ns,
            robot_x,
            robot_y,
            robot_z,
            gz_sim,
            spawn_robot,
            clock_bridge,
        ]
    )
