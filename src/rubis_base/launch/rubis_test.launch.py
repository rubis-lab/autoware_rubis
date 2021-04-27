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

    # lidar downsample
    scan_downsampler_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/voxel_grid.param.yaml')

    scan_downsampler_param = DeclareLaunchArgument(
        'scan_downsampler_param_file',
        default_value=scan_downsampler_param_file,
        description='Path to config file for lidar scan downsampler'
    )
    scan_downsampler = Node(
        package='voxel_grid_nodes',
        executable='voxel_grid_node_exe',
        namespace='lidars',
        name='voxel_grid_cloud_node',
        parameters=[LaunchConfiguration('scan_downsampler_param_file')],
        remappings=[
            ("points_in", "points_fused"),
            ("points_downsampled", "points_fused_downsampled")
        ]
    )

    # ray ground classifier
    ray_ground_classifier_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/ray_ground_classifier.param.yaml')
    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )
    ray_ground_classifier = Node(
        package='ray_ground_classifier_nodes',
        executable='ray_ground_classifier_cloud_node_exe',
        namespace='perception',
        parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
        remappings=[("points_in", "/lidars/points_fused")]
    )

    # ndt_localizer
    ndt_localizer_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/ndt_localizer.param.yaml')
    ndt_localizer_param = DeclareLaunchArgument(
        'ndt_localizer_param_file',
        default_value=ndt_localizer_param_file,
        description='Path to config file for ndt localizer'
    )
    ndt_localizer = Node(
        package='ndt_nodes',
        executable='p2d_ndt_localizer_exe',
        namespace='localization',
        name='p2d_ndt_localizer_node',
        parameters=[LaunchConfiguration('ndt_localizer_param_file')],
        remappings=[
            ("points_in", "/lidars/points_fused_downsampled"),
            ("observation_republish", "/lidars/points_fused_viz"),
        ]
    )

    return LaunchDescription([
        # point_cloud_transform (lidar)
        pc_filter_transform_lidar_front_param,
        filter_transform_vlp16_front,
        pc_filter_transform_lidar_rear_param,
        filter_transform_vlp16_rear,

        # downsample (lidar)
        scan_downsampler_param,
        scan_downsampler,

        # ray ground classifier
        ray_ground_classifier_param,
        ray_ground_classifier,

        # point_cloud_fusion (lidar)
        point_cloud_fusion_node,

        # ndt_localizer
        ndt_localizer_param,
        ndt_localizer,
    ])
