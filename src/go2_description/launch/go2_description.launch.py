"""
Unitree Go2 single-robot description launch file.

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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')

    # Launch arguments
    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation time')
    namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level node namespace')
    prefix_cmd = DeclareLaunchArgument(
        name='prefix',
        default_value='',
        description='Frame prefix')
    ld.add_action(use_sim_time_cmd)
    ld.add_action(namespace_cmd)
    ld.add_action(prefix_cmd)

    # Parse URDF file
    urdf = os.path.join(
        get_package_share_directory('go2_description'),
        'urdf',
        'go2.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='both',
        emulate_tty=True,
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': prefix,
            'use_sim_time': use_sim_time}])
    ld.add_action(robot_state_publisher)

    # joint_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        name='joint_state_publisher',
        output='both',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(joint_state_publisher)

    return ld
