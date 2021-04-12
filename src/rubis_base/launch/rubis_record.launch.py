# Copyright 2021 RUBIS

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """RUBIS launch description"""

    recordreplay_planner_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/recordreplay_planner.param.yaml')

    recordreplay_planner_param = DeclareLaunchArgument(
        'recordreplay_planner_param_file',
        default_value=recordreplay_planner_param_file,
        description='Path to config file for record/replay planner'
    )

    recordreplay_planner_node = Node(
        package='recordreplay_planner_nodes',
        executable='recordreplay_planner_node_exe',
        name='recordreplay_planner',
        namespace='planning',
        output='screen',
        parameters=[
            LaunchConfiguration('recordreplay_planner_param_file')
        ],
        remappings=[
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('planned_trajectory', '/planning/trajectory'),
        ]
    )
        # ('obstacle_bounding_boxes', '/perception/lidar_bounding_boxes'),


    return LaunchDescription([
        recordreplay_planner_param,
        recordreplay_planner_node,
    ])
