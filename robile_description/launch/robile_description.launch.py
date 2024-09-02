from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from robile_description import Robile


def launch_args(context):

    arguments = []

    arguments.append(
        DeclareLaunchArgument(
            "robot_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("robile_description"),
                    "config",
                    "default.yml"
                ]
            ),
            description="Robot configuration file with the bricks layout."
        )
    )

    return arguments


def launch_setup(context):

    robot_config = LaunchConfiguration("robot_config")

    robile = Robile(robot_config.perform(context))

    robot_description = {
        "robot_description": robile.description
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_node
    ]


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
