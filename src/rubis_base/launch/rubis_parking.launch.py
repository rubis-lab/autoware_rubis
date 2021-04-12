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

    # lanelet
    lanelet2_map_provider_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/lanelet2_map_provider.param.yaml')
    lanelet2_map_provider_param = DeclareLaunchArgument(
        'lanelet2_map_provider_param_file',
        default_value=lanelet2_map_provider_param_file,
        description='Path to parameter file for Lanelet2 Map Provider'
    )
    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_provider_exe',
        namespace='had_maps',
        name='lanelet2_map_provider_node',
        parameters=[LaunchConfiguration('lanelet2_map_provider_param_file')]
    )
    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_visualizer_exe',
        name='lanelet2_map_visualizer_node',
        namespace='had_maps'
    )


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

        global_planner = Node(
        package='lanelet2_global_planner_nodes',
        name='lanelet2_global_planner_node',
        namespace='planning',
        executable='lanelet2_global_planner_node_exe',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state')]
    )

    lane_planner_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/lane_planner.param.yaml')
    lane_planner_param = DeclareLaunchArgument(
        'lane_planner_param_file',
        default_value=lane_planner_param_file,
        description='Path to parameter file for lane planner'
    )
    lane_planner = Node(
        package='lane_planner_nodes',
        name='lane_planner_node',
        namespace='planning',
        executable='lane_planner_node_exe',
        parameters=[LaunchConfiguration('lane_planner_param_file')],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )

    behavior_planner_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/behavior_planner.param.yaml')
    behavior_planner_param = DeclareLaunchArgument(
        'behavior_planner_param_file',
        default_value=behavior_planner_param_file,
        description='Path to paramter file for behavior planner'
    )
    behavior_planner = Node(
        package='behavior_planner_nodes',
        name='behavior_planner_node',
        namespace='planning',
        executable='behavior_planner_node_exe',
        parameters=[LaunchConfiguration('behavior_planner_param_file')],
        output='screen',
        remappings=[
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('route', 'global_path'),
            ('vehicle_state_report', '/vehicle/state_report'),
            ('vehicle_state_command', '/vehicle/state_command')
        ]
    )

    off_map_obstacles_filter_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/off_map_obstacles_filter.param.yaml')
    off_map_obstacles_filter_param = DeclareLaunchArgument(
        'off_map_obstacles_filter_param_file',
        default_value=off_map_obstacles_filter_param_file,
        description='Path to parameter file for off-map obstacle filter'
    )
    off_map_obstacles_filter = Node(
        package='off_map_obstacles_filter_nodes',
        name='off_map_obstacles_filter_node',
        namespace='perception',
        executable='off_map_obstacles_filter_nodes_exe',
        parameters=[LaunchConfiguration('off_map_obstacles_filter_param_file')],
        output='screen',
        remappings=[
            ('bounding_boxes_in', 'lidar_bounding_boxes'),
            ('bounding_boxes_out', 'lidar_bounding_boxes_filtered'),
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
        ]
    )

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('rubis_base'),
            '/launch/rubis_base.launch.py']),
        launch_arguments={}.items()
    )

    return LaunchDescription([
        core_launch,

        # lanelet
        lanelet2_map_provider_param,
        lanelet2_map_provider,
        lanelet2_map_visualizer,

        parking_planner_param,
        parking_planner,

        global_planner,

        lane_planner_param,
        lane_planner,

        behavior_planner_param,
        behavior_planner,

        off_map_obstacles_filter_param,
        off_map_obstacles_filter,

    )]