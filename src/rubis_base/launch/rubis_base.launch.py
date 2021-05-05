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

    # urdf file for lexus_rx_450h
    urdf_path = os.path.join(
        get_package_share_directory('rubis_base'),
        'data/lexus_rx_450h.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()
    # use urdf/lexus_rx_450h_pcap for pcap file.
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )

    ######## map_publisher
    map_publisher_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
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

    # # map downsampler paramter file definition
    # ndt_map_downsampler_file_path = os.path.join(
    #     get_package_share_directory('ndt_nodes'),
    #     'param',
    #     'pcl_map_voxel_grid_downsample.param.yaml')
    # map_downsampler_param_file = LaunchConfiguration(
    #     'params', default=[ndt_map_downsampler_file_path])

    # # map downsample node execution definition
    # map_downsampler_node_runner = Node(
    #     package='voxel_grid_nodes',
    #     executable='voxel_grid_node_exe',
    #     parameters=[map_downsampler_param_file],
    #     remappings=[
    #         ("points_in", "viz_ndt_map"),
    #         ("points_downsampled", "viz_ndt_map_downsampled")
    #     ])

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
        'param/point_cloud_fusion.param.yaml')

    point_cloud_fusion_node_param = DeclareLaunchArgument(
        'point_cloud_fusion_node_param_file',
        default_value=point_cloud_fusion_node_param_file,
        description='Path to point_cloud_fusion_node_param_file'
    )

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
        'param/scan_downsampler_ms3.param.yaml')

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
            ("points_in", "points_nonground")
        ]
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

    object_collision_estimator_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/object_collision_estimator.param.yaml')
    object_collision_estimator_param = DeclareLaunchArgument(
        'object_collision_estimator_param_file',
        default_value=object_collision_estimator_param_file,
        description='Path to paramter file for object collision estimator'
    )
    object_collision_estimator = Node(
        package='object_collision_estimator_nodes',
        name='object_collision_estimator_node',
        namespace='planning',
        executable='object_collision_estimator_node_exe',
        parameters=[LaunchConfiguration('object_collision_estimator_param_file')],
        remappings=[
            ('obstacle_topic', '/perception/lidar_bounding_boxes'),
        ]
    )

    # mpc
    mpc_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/mpc.param.yaml')
    mpc_param = DeclareLaunchArgument(
        'mpc_param_file',
        default_value=mpc_param_file,
        description='Path to config file for MPC'
    )
    mpc = Node(
        package='mpc_controller_nodes',
        executable='mpc_controller_node_exe',
        name='mpc_controller',
        namespace='control',
        parameters=[LaunchConfiguration('mpc_param_file')],
        # remappings=[

        # ]
    )

    pure_pursuit_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/pure_pursuit.param.yaml')
    pure_pursuit_param = DeclareLaunchArgument(
        'pure_pursuit_param_file',
        default_value=pure_pursuit_param_file,
        description='Path to config file to Pure Pursuit Controller'
    )
    pure_pursuit = Node(
        package="pure_pursuit_nodes",
        executable="pure_pursuit_node_exe",
        # namespace="control",
        name="pure_pursuit_node",
        output="screen",
        parameters=[LaunchConfiguration("pure_pursuit_param_file"), {}],
        remappings=[
            ("current_pose", "/vehicle/vehicle_kinematic_state"),
            ("trajectory", "/planning/trajectory"),
            ("ctrl_cmd", "/vehicle/vehicle_command"),
            ("ctrl_diag", "/control/control_diagnostic"),
        ],
    )

    ######## lgsvl
    lgsvl_param_file = os.path.join(
        get_package_share_directory('rubis_base'),
        'param/lgsvl_interface.param.yaml')
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_param_file,
        description='Path to config file for LGSVL Interface'
    )
    lgsvl_interface = Node(
        package='lgsvl_interface',
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        output='screen',
        parameters=[
          LaunchConfiguration('lgsvl_interface_param_file'),
          {"lgsvl.publish_tf": "true"}
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ]
    )

    ######## rviz2
    rviz_cfg_path = os.path.join(
        get_package_share_directory('rubis_base'),
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

    ######## odom
    # This is a hack to make the mapper purely rely on the ndt localizer without using any
    # odometry source.
    # TODO(yunus.caliskan): Revisit after #476
    odom_bl_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    return LaunchDescription([
        # robot_state_publisher(urdf)
        urdf_publisher,

        # map_publisher
        map_publisher_param,
        map_publisher,
        # map_downsampler_node_runner,

        # lanelet
        lanelet2_map_provider_param,
        lanelet2_map_provider,
        lanelet2_map_visualizer,

        # point_cloud_transform (lidar)
        pc_filter_transform_lidar_front_param,
        filter_transform_vlp16_front,
        pc_filter_transform_lidar_rear_param,
        filter_transform_vlp16_rear,

        # point_cloud_fusion (lidar)
        point_cloud_fusion_node,

        # downsample (lidar)
        scan_downsampler_param,
        scan_downsampler,

        # ray ground classifier
        ray_ground_classifier_param,
        ray_ground_classifier,

        # euclidean cluster
        euclidean_cluster_param,
        euclidean_clustering,

        # object detection
        object_collision_estimator_param,
        object_collision_estimator,

        # ndt_localizer
        ndt_localizer_param,
        ndt_localizer,

        # odom hack
        odom_bl_publisher,

        # mpc
        mpc_param,
        mpc,

        # pure_pursuit_param,
        # pure_pursuit,

        # lgsvl
        lgsvl_interface_param,
        lgsvl_interface,

        rviz2,

    ])
