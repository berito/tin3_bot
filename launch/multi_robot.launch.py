# Multi-robot launch file for tin3_bot
# Spawns N robots with unique namespaces
# Usage: ros2 launch tin3_bot multi_robot.launch.py num_robots:=4

import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_robots(context: LaunchContext, num_robots: LaunchConfiguration):
    """Dynamically create robot spawn actions based on num_robots parameter"""
    
    pkg_tin3_bot = get_package_share_directory("tin3_bot")
    num = int(context.perform_substitution(num_robots))
    
    # Grid layout: robots spaced 2m apart
    spawn_actions = []
    cols = 4  # robots per row
    spacing = 2.0
    
    for i in range(num):
        row = i // cols
        col = i % cols
        x = col * spacing
        y = row * spacing
        ns = f"robot{i + 1}"
        
        spawn = TimerAction(
            period=float(i) * 2.0 + 3.0,  # Stagger spawns, start after 3s
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_tin3_bot, "launch", "spawn_robot.launch.py")
                    ),
                    launch_arguments={
                        "robot_ns": ns,
                        "x": str(x),
                        "y": str(y),
                        "z": "0.5",
                    }.items(),
                )
            ]
        )
        spawn_actions.append(spawn)
    
    return spawn_actions


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

    num_robots_arg = DeclareLaunchArgument(
        "num_robots",
        default_value="2",
        description="Number of robots to spawn",
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
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

    # Dynamic robot spawning
    spawn_robots_action = OpaqueFunction(
        function=spawn_robots,
        args=[LaunchConfiguration("num_robots")]
    )

    return LaunchDescription(
        [
            gz_resource_path,
            sim_world,
            num_robots_arg,
            gz_sim,
            clock_bridge,
            spawn_robots_action,
        ]
    )