#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Darby Lim

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    rviz_config_dir = os.path.join(
            get_package_share_directory('bayesian-sensor-fusion'),
            'launch',
            'config.rviz')

    ekf_yaml_path = os.path.join(
            get_package_share_directory('bayesian-sensor-fusion'),
            'params',
            'ekf_params.yaml')

    amcl_yaml_path = os.path.join(
            get_package_share_directory('bayesian-sensor-fusion'),
            'params',
            'amcl_params.yaml')

    rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_yaml_path])

    nav2_amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='',
            output='screen',
            parameters=[amcl_yaml_path])

    gtsam_node = Node(
            package='bayesian-sensor-fusion',
            executable='talker',
            name='talker',
            output='screen')

    nav2_lifecycle =Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_amcl',
            output='screen',
            parameters=[{'node_names' : ["amcl"]},
                        {'autostart': True},
                        {'use_sim_time': True}])


    ld = LaunchDescription()

    ld.add_action(ekf_node)
    ld.add_action(nav2_amcl_node)
    ld.add_action(rviz2_node)
    ld.add_action(nav2_lifecycle)
    ld.add_action(gtsam_node) 

    return ld
