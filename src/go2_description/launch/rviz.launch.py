"""
RViz launch file.

Roberto Masocco <r.masocco@dotxautomation.com>

July 26, 2024
"""

# Copyright 2024 dotX Automation s.r.l.
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

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    rviz_config = os.path.join(
        get_package_share_directory('go2_description'),
        'rviz',
        'go2.rviz')

    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        emulate_tty=True,
        output='screen')
    ld.add_action(rviz)

    return ld
