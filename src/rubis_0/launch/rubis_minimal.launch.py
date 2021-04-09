# Copyright 2021 RUBIS

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """RUBIS launch description"""

    ######## robot_state_publisher
    # will publish to /tf_static

    # urdf file for lexus_rx_450h
    urdf_path = os.path.join(
        get_package_share_directory('rubis_0'),
        'data/lexus_rx_450h.urdf')
    # use urdf/lexus_rx_450h_pcap for pcap file.
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[str(urdf_path)]
    )

    ######## map_publisher
    map_publisher_param_file = os.path.join(
        get_package_share_directory('rubis_0'),
        'param/map_publisher.param.yaml')
    
    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=map_publisher_param_file,
        description='Path to config file for Map Publisher'
    )
    map_publisher = Node(
        package='ndt_nodes',
        executable='ndt_map_publisher_exe',
        namespace='localization',
        parameters=[LaunchConfiguration('map_publisher_param_file')]
    )

    ######## rviz2
    rviz_cfg_path = os.path.join(
        get_package_share_directory('rubis_0'),
        'param/rubis-rviz.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        # condition=IfCondition(LaunchConfiguration('with_rviz')),
        remappings=[("initialpose", "/localization/initialpose"),
            ("goal_pose", "/planning/goal_pose")],
    )


    return LaunchDescription([
        # robot_state_publisher(urdf)
        urdf_publisher,

        # map_publisher
        map_publisher_param,
        map_publisher,
        # map_downsampler_node_runner,

        # rviz2,
    ])
