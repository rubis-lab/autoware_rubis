# Copyright 2021 RUBIS

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """RUBIS launch description"""

    rubis_node = Node(
        package='rubis_0',
        executable='rubis_0_node_exe',
        name='rubis_0',
        arguments=[]
    )

    return LaunchDescription([
        rubis_node,
    ])
