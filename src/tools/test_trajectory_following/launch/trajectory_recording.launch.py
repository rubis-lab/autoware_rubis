# Copyright 2020 The Autoware Foundation.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Testing of mpc_controller in LGSVL simulation using recordreplay planner."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    """
    Launch nodes with params to test record_replay_planner with mpc_controller in simulation.

    mpc_controller + LGSVL + recordreplay planner + lidar osbtacle ddetection
    """
    # --------------------------------- Params -------------------------------
    joy_translator_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='param/logitech_f310.param.yaml')
    lgsvl_interface_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='param/lgsvl_interface.param.yaml')
    pc_filter_transform_param_file = get_share_file(
        package_name='point_cloud_filter_transform_nodes',
        file_name='param/vlp16_sim_lexus_filter_transform.param.yaml')
    ray_ground_classifier_param_file = get_share_file(
        package_name='autoware_auto_avp_demo', file_name='param/ray_ground_classifier.param.yaml')
    euclidean_cluster_param_file = get_share_file(
        package_name='autoware_auto_avp_demo', file_name='param/euclidean_cluster.param.yaml')
    mpc_controller_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='param/mpc_controller.param.yaml')
    rviz_cfg_path = get_share_file(
        package_name='test_trajectory_following', file_name='config/mpc_cotrols.rviz')
    urdf_path = get_share_file(
        package_name='lexus_rx_450h_description', file_name='urdf/lexus_rx_450h.urdf')

    # --------------------------------- Arguments -------------------------------
    joy_translator_param = DeclareLaunchArgument(
        'joy_translator_param_file',
        default_value=joy_translator_param_file,
        description='Path to config file for joystick translator'
    )
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_interface_param_file,
        description='Path to config file for LGSVL Interface'
    )
    pc_filter_transform_param = DeclareLaunchArgument(
        'pc_filter_transform_param_file',
        default_value=pc_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )
    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )
    euclidean_cluster_param = DeclareLaunchArgument(
        'euclidean_cluster_param_file',
        default_value=euclidean_cluster_param_file,
        description='Path to config file for Euclidean Clustering'
    )
    mpc_controller_param = DeclareLaunchArgument(
        'mpc_controller_param_file',
        default_value=mpc_controller_param_file,
        description='Path to config file for MPC Controller'
    )
    with_mpc_param = DeclareLaunchArgument(
        'with_mpc',
        default_value='True',
        description='Enable mpc controller'
    )
    with_obstacle_param = DeclareLaunchArgument(
        'with_obstacle',
        default_value='True',
        description='Enable obstacle detection stack \
            (filter_transform + ray_ground_classifier + euclidean_clustering)'
    )

    # -------------------------------- Nodes-----------------------------------

    joystick_launch_file_path = get_share_file(
        package_name='joystick_vehicle_interface',
        file_name='launch/joystick_vehicle_interface.launch.py')
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_file_path),
        launch_arguments=[
            ("joy_translator_param", LaunchConfiguration('joy_translator_param_file'))]
    )
    lgsvl_interface = Node(
        package='lgsvl_interface',
        node_executable='lgsvl_interface_exe',
        node_namespace='vehicle',
        output='screen',
        parameters=[LaunchConfiguration('lgsvl_interface_param_file')]
    )
    recordreplay_planner_path = get_share_file(
        package_name='recordreplay_planner_node',
        file_name='launch/recordreplay_planner_node.launch.py')
    recordreplay_planner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(recordreplay_planner_path)
    )
    joy_ctrl_record_replay_traj = Node(
        package="test_trajectory_following",
        node_executable="joy_ctrl_record_replay_traj.py",
        node_name="joy_ctrl_record_replay_traj",
        parameters=[],
        remappings=[
            ("vehicle_kinematic_state", "/vehicle/vehicle_kinematic_state")
        ],
        output='screen',
    )
    filter_transform_vlp16_front = Node(
        package='point_cloud_filter_transform_nodes',
        node_executable='point_cloud_filter_transform_node_exe',
        node_name='filter_transform_vlp16_front',
        node_namespace='lidar_front',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        condition=IfCondition(LaunchConfiguration('with_obstacle'))
    )
    filter_transform_vlp16_rear = Node(
        package='point_cloud_filter_transform_nodes',
        node_executable='point_cloud_filter_transform_node_exe',
        node_name='filter_transform_vlp16_rear',
        node_namespace='lidar_rear',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        condition=IfCondition(LaunchConfiguration('with_obstacle'))
    )
    ray_ground_classifier = Node(
        package='ray_ground_classifier_nodes',
        node_executable='ray_ground_classifier_cloud_node_exe',
        node_namespace='perception',
        parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
        condition=IfCondition(LaunchConfiguration('with_obstacle'))
    )
    euclidean_clustering = Node(
        package='euclidean_cluster_nodes',
        node_executable='euclidean_cluster_exe',
        node_namespace='perception',
        parameters=[LaunchConfiguration('euclidean_cluster_param_file')],
        condition=IfCondition(LaunchConfiguration('with_obstacle'))
    )
    mpc_controller_node = Node(
        package="mpc_controller_node",
        node_executable="mpc_controller_node_exe",
        node_name="mpc_controller",
        node_namespace='control',
        parameters=[LaunchConfiguration('mpc_controller_param_file')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_mpc'))
    )
    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', str(rviz_cfg_path)]
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        arguments=[str(urdf_path)]
    )
    return LaunchDescription([
        joy_translator_param,
        joystick,
        lgsvl_interface_param,
        lgsvl_interface,
        with_obstacle_param,
        pc_filter_transform_param,
        ray_ground_classifier_param,
        euclidean_cluster_param,
        filter_transform_vlp16_front,
        filter_transform_vlp16_rear,
        ray_ground_classifier,
        euclidean_clustering,
        recordreplay_planner_node,
        joy_ctrl_record_replay_traj,
        with_mpc_param,
        mpc_controller_param,
        mpc_controller_node,
        urdf_publisher,
        rviz2
    ])
