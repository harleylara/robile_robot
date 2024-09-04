from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_args(context):

    # set of available arguments
    declared_args = []

    declared_args.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RVIZ2 while launching."
        )
    )

    # TODO: add rviz config file

    declared_args.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="robile",
            description="Robot name."
        )
    )

    return declared_args


def launch_setup(context):

    launch_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ]
        )
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        arguments=['-topic', "robot_description",
                   "-x", "0.0",
                   "-y", "0.0",
                   "-z", "0.54", 
                   "-entity",  LaunchConfiguration("robot_name")],
    )

    return [
        launch_gazebo,
        spawn_robot
    ]


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_args))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
