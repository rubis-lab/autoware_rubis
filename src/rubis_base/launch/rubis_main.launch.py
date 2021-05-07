# Copyright 2021 RUBIS

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    'RUBIS launch description'

    # ######## robot_state_publisher
    # # will publish to /tf_static



    ######## point_cloud_transform (lidar)
    num_lidars = 4
    pc_filter_param_files = [
        os.path.join(get_package_share_directory('rubis_base'),
            'param/pc_filter_transform_lidar_' + str(lidar_idx) + '.param.yaml')
            for lidar_idx in range(num_lidars)]

    pc_filter_params = [
        DeclareLaunchArgument('pc_filter_transform_lidar_' + str(lidar_idx) + '_param_file',
            default_value=pc_filter_param_files[lidar_idx],
            description='Path to config file for Point Cloud Filter/Transform Nodes')
            for lidar_idx in range(num_lidars)]

    pc_filter_nodes = [
        Node(
            package='point_cloud_filter_transform_nodes',
            executable='point_cloud_filter_transform_node_exe',
            name='point_cloud_filter_transform_' + str(lidar_idx),
            namespace='lidar_' + str(lidar_idx),
            parameters=[LaunchConfiguration(
                'pc_filter_transform_lidar_' + str(lidar_idx) + '_param_file')],
            remappings=[('points_in', 'points_raw')])
            for lidar_idx in range(num_lidars)]

    # point cloud fusion
    point_cloud_fusion_node_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/point_cloud_fusion.param.yaml')

    point_cloud_fusion_node_param = DeclareLaunchArgument(
        'point_cloud_fusion_node_param_file',
        default_value=point_cloud_fusion_node_param_file,
        description='Path to point_cloud_fusion_node_param_file'
    )

    point_cloud_fusion_node = Node(
        package='point_cloud_fusion_nodes',
        executable='pointcloud_fusion_node_exe',
        # namespace='lidars',
        parameters=[point_cloud_fusion_node_param_file],
        remappings= [('output_topic', 'lidars/points_fused')] +
            [('input_topic' + str(lidar_idx),
                'lidar_' + str(lidar_idx) + '/points_filtered')
            for lidar_idx in range(num_lidars)]
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
        remappings=[('points_in', '/lidars/points_fused')]
    )

    # euclidean_cluster
    euclidean_cluster_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/euclidean_cluster.param.yaml')
    euclidean_cluster_param = DeclareLaunchArgument(
        'euclidean_cluster_param_file',
        default_value=euclidean_cluster_param_file,
        description='Path to config file for Euclidean Clustering'
    )
    euclidean_clustering = Node(
        package='euclidean_cluster_nodes',
        executable='euclidean_cluster_node_exe',
        namespace='perception',
        parameters=[LaunchConfiguration('euclidean_cluster_param_file')],
        remappings=[
            ('points_in', '/perception/points_nonground')
        ]
    )

    # rubis_detect
    rubis_detect_param_file = os.path.join(
        get_package_share_directory('rubis_detect'),
        'param/rubis_detect.param.yaml')
    rubis_detect_param = DeclareLaunchArgument(
        'rubis_detect_param_file',
        default_value=rubis_detect_param_file,
        description='Path to config file to Rubis Detect'
    )

    rubis_detect = Node(
        package='rubis_detect',
        executable='rubis_detect_node_exe',
        name='rubis_detect_node',
        output='screen',
        parameters=[LaunchConfiguration('rubis_detect_param_file'), {}],
    )

    # rubis_drive
    rubis_drive_param_file = os.path.join(
        get_package_share_directory('rubis_drive'),
        'param/rubis_drive.param.yaml')
    rubis_drive_param = DeclareLaunchArgument(
        'rubis_drive_param_file',
        default_value=rubis_drive_param_file,
        description='Path to config file to Rubis Drive'
    )

    rubis_drive = Node(
        package='rubis_drive',
        executable='rubis_drive_node_exe',
        name='rubis_drive_node',
        output='screen',
        parameters=[LaunchConfiguration('rubis_drive_param_file'), {}],
    )

    return LaunchDescription(
        pc_filter_params +
        pc_filter_nodes +
        [
        # point_cloud_transform (lidar)
        # pc_filter_transform_lidar_0_param,
        # filter_transform_vlp16_0,
        # pc_filter_transform_lidar_1_param,
        # filter_transform_vlp16_1,
        # pc_filter_transform_lidar_2_param,
        # filter_transform_vlp16_2,
        # pc_filter_transform_lidar_3_param,
        # filter_transform_vlp16_3,
        # pc_filter_transform_lidar_4_param,
        # filter_transform_vlp16_4,
        # pc_filter_transform_lidar_5_param,
        # filter_transform_vlp16_5,
        # pc_filter_transform_lidar_6_param,
        # filter_transform_vlp16_6,
        # pc_filter_transform_lidar_7_param,
        # filter_transform_vlp16_7,

        # point_cloud_fusion (lidar)
        point_cloud_fusion_node_param,
        point_cloud_fusion_node,

        # ray ground classifier
        ray_ground_classifier_param,
        ray_ground_classifier,

        # euclidean cluster
        euclidean_cluster_param,
        euclidean_clustering,

        # rubis_detect
        # rubis_detect_param,
        # rubis_detect,

        # # rubis_drive
        # rubis_drive_param,
        # rubis_drive,
    ])
