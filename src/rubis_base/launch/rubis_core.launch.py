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


        # odom hack
        odom_bl_publisher,

        # lgsvl
        lgsvl_interface_param,
        lgsvl_interface,

        rviz2,

    ])
