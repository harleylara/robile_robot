from typing import Union
from pathlib import Path
from dataclasses import dataclass

import yaml
import xacro

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
class GenericBrick:
    brick_type: str
    name: str
    parent: str
    pos: Position
    rot: Rotation
    movable_joints: Union[bool, None] = None

    def __repr__(self) -> str:
        return f"""
    <xacro:{self.brick_type} name="{self.name}" parent="{self.parent}" {"" if self.movable_joints is None else 'movable_joints="$(arg movable_joints)"'}>
    <origin xyz="{self.pos.x} {self.pos.y} {self.pos.z}" rpy="{self.rot.roll} {self.rot.pitch} {self.rot.yaw}"/>
    </xacro:{self.brick_type}>
        """

class Robile:

    __BRICK_WIDTH = 0.233
    __BRICK_HEIGHT = 0.233
    __MOVABLES_BRICKS = ["active_wheel", "passive_wheel"]
    __NON_MOVABLES_BRICKS = ["cpu", "battery"]
    __SPECIAL_BRICKS = ["skip"]
    __PHYSICAL_BRICKS = __MOVABLES_BRICKS + __NON_MOVABLES_BRICKS
    __SUPPORTED_BRICKS = __PHYSICAL_BRICKS + __SPECIAL_BRICKS

    def __init__(self, config_file: Union[str, Path]) -> None:

        if isinstance(config_file, str):
            self.config_file = Path(config_file)
        elif isinstance(config_file, Path):
            self.config_file = config_file
        else:
            raise TypeError(f"Expected 'config_file' to be a string or Path, got {type(config_file).__name__}")

        if not self.config_file.is_file():
            raise FileNotFoundError(f"The file at {self.config_file} does not exist or is not a file.")
        
        with open(self.config_file, 'r') as file:
            data = yaml.safe_load(file)

        self.robot_name = str(data["robot_name"]).strip().lower()
        self.layout = data["layout"]

        self.__xacro = f"""<?xml version='1.0'?>
    <robot xmlns:xacro="http://ros.org/wiki/xacro" name="{self.robot_name}" >

    <xacro:arg name="movable_joints" default="true"/>

    <xacro:include filename="$(find robile_description)/urdf/bricks/active_wheel.urdf.xacro" />
    <xacro:include filename="$(find robile_description)/urdf/bricks/passive_wheel.urdf.xacro" />
    <xacro:include filename="$(find robile_description)/urdf/bricks/cpu.urdf.xacro" />
    <xacro:include filename="$(find robile_description)/urdf/bricks/battery.urdf.xacro" />

    <link name="base_link"/>

    <joint name="base_footprint_joint" type="fixed">
      <parent link="base_link"/>
      <child link="base_footprint"/>
      <origin xyz="0.0 0.0 -0.05" rpy="0 0 0"/>
    </joint>

    <link name="base_footprint"/>
    """
        self.__xacro_eof = "</robot>"

        # Defition of the final robot description
        # as URDF format
        self.__description: str = ""

        self.__process_config()


    def __process_config(self):

        rows = len(self.layout)
        cols = max(len(row) for row in self.layout)

        total_width = cols * self.__BRICK_WIDTH
        total_height = rows * self.__BRICK_HEIGHT

        center_x = total_width / 2
        center_y = total_height / 2

        for i, row in enumerate(self.layout):
            for j, brick_name in enumerate(row):
                brick_name = str(brick_name).strip().lower()
                x = j * self.__BRICK_HEIGHT - center_x + (self.__BRICK_WIDTH / 2)
                y = -(i * self.__BRICK_HEIGHT - center_y + (self.__BRICK_HEIGHT / 2))  # Negative because y decreases going up

                if brick_name in self.__SUPPORTED_BRICKS:
                    if brick_name in self.__PHYSICAL_BRICKS:
                        brick = GenericBrick(
                            brick_type=brick_name,
                            name=f"{brick_name}_{i}{j}",
                            parent="base_link",
                            pos=Position(x=x, y=y, z=0),
                            rot=Rotation(roll=0, pitch=0, yaw=0),
                            movable_joints=None if brick_name in self.__NON_MOVABLES_BRICKS else True
                        )
                        self.__xacro += str(brick)
                else:
                    raise ValueError(f"'{brick_name}' is not an supported brick. Supported bricks are {self.__SUPPORTED_BRICKS}.")

        self.__xacro += self.__xacro_eof
        # time to convert from XACRO to URDF

        xacro.init_stacks(self.__xacro)
        doc = xacro.parse(self.__xacro)
        xacro.process_doc(doc)
        self.__description = doc.toxml()

    @property
    def xacro(self):
        return self.__xacro

    @property
    def urdf(self):
        return self.__description

    @property
    def description(self):
        return self.__description
