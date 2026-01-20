# Spawn robot launch file for tin3_bot
# Can be used standalone or called from main launch with namespace for multi-robot

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def spawn_robot(
    context: LaunchContext,
    namespace: LaunchConfiguration,
    x_pos: LaunchConfiguration,
    y_pos: LaunchConfiguration,
    z_pos: LaunchConfiguration,
):
    pkg_project_description = get_package_share_directory("tin3_bot")
    robot_ns = context.perform_substitution(namespace)
    x_value = context.perform_substitution(x_pos)
    y_value = context.perform_substitution(y_pos)
    z_value = context.perform_substitution(z_pos)

    robot_desc = xacro.process(
        os.path.join(
            pkg_project_description,
            "urdf",
            "robot.urdf.xacro",
        ),
        mappings={"robot_ns": robot_ns},
    )

    if robot_ns == "":
        robot_gazebo_name = "tin3_bot"
        node_name_prefix = ""
    else:
        robot_gazebo_name = "tin3_bot_" + robot_ns
        node_name_prefix = robot_ns + "_"

    # Launch robot state publisher node
    robot_state_publisher = Node(
        namespace=robot_ns,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )

    # Spawn robot inside simulation
    spawn_entity = Node(
        namespace=robot_ns,
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_sim_create",
        output="both",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            robot_gazebo_name,
            "-x",
            x_value,
            "-y",
            y_value,
            "-z",
            z_value,
        ],
    )

    # Bridge ROS topics and Gazebo messages
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=node_name_prefix + "parameter_bridge",
        arguments=[
            robot_ns + "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            robot_ns + "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            robot_ns + "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            robot_ns + "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return [
        robot_state_publisher,
        spawn_entity,
        topic_bridge,
    ]


def generate_launch_description():
    name_argument = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )
    x_argument = DeclareLaunchArgument(
        "x",
        default_value="0.0",
        description="Spawn position X",
    )
    y_argument = DeclareLaunchArgument(
        "y",
        default_value="0.0",
        description="Spawn position Y",
    )
    z_argument = DeclareLaunchArgument(
        "z",
        default_value="0.5",
        description="Spawn position Z",
    )

    namespace = LaunchConfiguration("robot_ns")
    x_pos = LaunchConfiguration("x")
    y_pos = LaunchConfiguration("y")
    z_pos = LaunchConfiguration("z")

    return LaunchDescription(
        [
            name_argument,
            x_argument,
            y_argument,
            z_argument,
            OpaqueFunction(
                function=spawn_robot, args=[namespace, x_pos, y_pos, z_pos]
            ),
        ]
    )
