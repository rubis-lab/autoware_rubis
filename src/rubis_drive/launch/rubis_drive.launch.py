# Copyright 2021 the Autoware Foundation
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

"""
Example launch file for a new package.

Note: Does not work in ROS2 dashing!
"""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate launch description with a single component."""
    rubis_drive_param_file = os.path.join(
        get_package_share_directory('rubis_drive'),
        'param/rubis_drive.param.yaml')
    rubis_drive_param = DeclareLaunchArgument(
        'rubis_drive_param_file',
        default_value=rubis_drive_param_file,
        description='Path to config file to Rubis Drive'
    )

    rubis_drive = Node(
        package="rubis_drive",
        executable="rubis_drive_node_exe",
        name="rubis_drive_node",
        output="screen",
        parameters=[LaunchConfiguration("rubis_drive_param_file"), {}],
    )

    return LaunchDescription([
        rubis_drive_param,
        rubis_drive,
    ])
