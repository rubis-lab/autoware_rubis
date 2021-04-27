# Copyright 2021 RUBIS

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """RUBIS launch description"""

    # ######## robot_state_publisher
    # # will publish to /tf_static



    ######## point_cloud_transform (lidar)
    pc_filter_transform_lidar_front_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/pc_filter_transform_lidar_front.param.yaml')

    pc_filter_transform_lidar_front_param = DeclareLaunchArgument(
        'pc_filter_transform_lidar_front_param_file',
        default_value=pc_filter_transform_lidar_front_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )

    filter_transform_vlp16_front = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_front',
        namespace='lidar_front',
        parameters=[LaunchConfiguration('pc_filter_transform_lidar_front_param_file')],
        remappings=[("points_in", "points_raw")]
    )

    pc_filter_transform_lidar_rear_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/pc_filter_transform_lidar_rear.param.yaml')

    pc_filter_transform_lidar_rear_param = DeclareLaunchArgument(
        'pc_filter_transform_lidar_rear_param_file',
        default_value=pc_filter_transform_lidar_rear_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )
    filter_transform_vlp16_rear = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_rear',
        namespace='lidar_rear',
        parameters=[LaunchConfiguration('pc_filter_transform_lidar_rear_param_file')],
        remappings=[("points_in", "points_raw")]
    )

    # point cloud fusion
    point_cloud_fusion_node_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/vlp16_sim_lexus_pc_fusion.param.yaml')

    # point_cloud_fusion_node_param = DeclareLaunchArgument(
    #     'point_cloud_fusion_node_param_file',
    #     default_value=point_cloud_fusion_node_param_file,
    #     description='Path to point_cloud_fusion_node_param_file'
    # )

    point_cloud_fusion_node = Node(
        package='point_cloud_fusion_nodes',
        executable='pointcloud_fusion_node_exe',
        namespace='lidars',
        parameters=[point_cloud_fusion_node_param_file],
        remappings=[
            ("output_topic", "points_fused"),
            ("input_topic1", "/lidar_front/points_filtered"),
            ("input_topic2", "/lidar_rear/points_filtered")
        ]
    )

    return LaunchDescription([
        # point_cloud_transform (lidar)
        pc_filter_transform_lidar_front_param,
        filter_transform_vlp16_front,
        pc_filter_transform_lidar_rear_param,
        filter_transform_vlp16_rear,

        # point_cloud_fusion (lidar)
        point_cloud_fusion_node,
    ])
