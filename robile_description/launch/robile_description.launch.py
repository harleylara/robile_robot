from dataclasses import dataclass

#import yaml
import xacro

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

@dataclass
class Position:
    x: float
    y: float
    z: float

@dataclass
class Rotation:
    roll: float
    pitch: float
    yaw: float

@dataclass
class GenericWheel:
    wheel_type: str
    name: str
    parent: str
    pos: Position
    rot: Rotation

    def __repr__(self) -> str:
        return f"""
        <xacro:{self.wheel_type} name="{self.name}" parent="{self.parent}" movable_joints="$(arg movable_joints)">
        <origin xyz="{self.pos.x} {self.pos.y} {self.pos.z}" rpy="{self.rot.roll} {self.rot.pitch} {self.rot.yaw}"/>
        </xacro:{self.wheel_type}>
        """

@dataclass
class GenericBrick:
    brick_type: str
    name: str
    parent: str
    pos: Position
    rot: Rotation

    def __repr__(self) -> str:
        return f"""
        <xacro:{self.brick_type} name="{self.name}" parent="{self.parent}">
        <origin xyz="{self.pos.x} {self.pos.y} {self.pos.z}" rpy="{self.rot.roll} {self.rot.pitch} {self.rot.yaw}"/>
        </xacro:{self.brick_type}>
        """


def launch_args(context):

    arguments = []

    arguments.append(
        DeclareLaunchArgument(
            "robot_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("robile_description"),
                    "config",
                    "harley.yml"
                ]
            ),
            description="Robot configuration file with the bricks layout."
        )
    )

    return arguments


def launch_setup(context):

    robot_config = LaunchConfiguration("robot_config")

    full_content = """<?xml version='1.0'?>
    <robot xmlns:xacro="http://ros.org/wiki/xacro" name="simple_config" >

    <xacro:arg name="movable_joints" default="true"/>

    <xacro:include filename="$(find robile_description)/urdf/active_wheel.urdf.xacro" />
    <xacro:include filename="$(find robile_description)/urdf/passive_wheel.urdf.xacro" />
    <xacro:include filename="$(find robile_description)/urdf/cpu.urdf.xacro" />
    <xacro:include filename="$(find robile_description)/urdf/battery.urdf.xacro" />

    <link name="base_link"/>

    <joint name="base_footprint_joint" type="fixed">
      <parent link="base_link"/>
      <child link="base_footprint"/>
      <origin xyz="0.0 0.0 -0.01" rpy="0 0 0"/>
    </joint>

    <link name="base_footprint"/>
    """

    for count, brick in enumerate(["active_wheel", "passive_wheel", "cpu", "battery"]):

        brick_name = brick.strip().lower()

        if brick_name == "active_wheel":
            brick = GenericWheel(
                wheel_type="active_wheel",
                name=f"robile_{count}",
                parent="base_link",
                pos=Position(x=count*0.233, y=0, z=0),
                rot=Rotation(roll=0, pitch=0, yaw=0)
            )
        if brick_name == "passive_wheel":
            brick = GenericWheel(
                wheel_type="passive_wheel",
                name=f"robile_{count}",
                parent="base_link",
                pos=Position(x=count*0.233, y=0, z=0),
                rot=Rotation(roll=0, pitch=0, yaw=0)
            )
        if brick_name == "cpu":
            brick = GenericBrick(
                brick_type="cpu",
                name=f"robile_{count}",
                parent="base_link",
                pos=Position(x=count*0.233, y=0, z=0),
                rot=Rotation(roll=0, pitch=0, yaw=0)
            )
        if brick_name == "battery":
            brick = GenericBrick(
                brick_type="battery",
                name=f"robile_{count}",
                parent="base_link",
                pos=Position(x=count*0.233, y=0, z=0),
                rot=Rotation(roll=0, pitch=0, yaw=0)
            )



        full_content += str(brick)

 
    full_content += "</robot>"
    xacro.init_stacks(full_content)
    doc = xacro.parse(full_content)
    xacro.process_doc(doc)
    urdf = doc.toxml()

    # with open(robot_config.perform(context), 'r') as file:
    #     data = yaml.safe_load(file)
    #
    # # number of bricks in the y-axis
    # n_y_bricks = len(data["layout"])
    #
    # print(n_y_bricks)

    urdf_path = PathJoinSubstitution([FindPackageShare("robile_description"), "urdf", "robot.urdf.xacro"])

    # robot_description = {
    #     "robot_description": xacro.process_file(urdf_path.perform(context)).toxml()
    # }

    # urdf = xacro.process_file(urdf_path.perform(context)).toxml()

    robot_description = {
        "robot_description": urdf
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
