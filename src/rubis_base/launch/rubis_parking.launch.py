# Copyright 2021 RUBIS

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    """RUBIS launch description"""

    # ######## robot_state_publisher
    # # will publish to /tf_static

    parking_planner_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/parking_planner.param.yaml')
    parking_planner_param = DeclareLaunchArgument(
        'parking_planner_param_file',
        default_value=parking_planner_param_file,
        description='Path to paramter file for parking planner'
    )
    parking_planner = Node(
        package='parking_planner_nodes',
        name='parking_planner_node',
        namespace='planning',
        executable='parking_planner_node_exe',
        parameters=[LaunchConfiguration('parking_planner_param_file')],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('rubis_base'),
            '/launch/rubis_base.launch.py']),
        launch_arguments={}.items()
    )

    return LaunchDescription([
        core_launch,
        parking_planner_param,
        parking_planner,
    )]