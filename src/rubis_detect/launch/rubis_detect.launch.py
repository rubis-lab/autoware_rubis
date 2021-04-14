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

import ament_index_python
import launch
import launch_ros.actions


def generate_launch_description():
    """Generate launch description with a single component."""
    rubis_detect_node = launch_ros.actions.Node(
        package='rubis_detect'
        executable='rubis_detect_exe',
        name='rubis_detect_node',
        namespace='',
        output='screen',
        parameters=[
            "{}/param/rubis_detect.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "rubis_detect"
                )
            ),
        ]
    )
    ld = launch.LaunchDescription([rubis_detect_node])
    return ld
