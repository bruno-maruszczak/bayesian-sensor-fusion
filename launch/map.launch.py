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
    map_yaml_path = os.path.join(
        get_package_share_directory('bayesian-sensor-fusion'),
        'maps',
        'map.yaml')
    
    map_server_node = LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'topic_name': 'map',
                'use_sim_ime': True,
                'yaml_filename': map_yaml_path
            }]
        )
    
    nav2_lifecycle = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'node_names' : ['map_server']},
                        {'autostart': True}])

    map_odom_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', '/map', '/odom'],
            output='screen')
    
    ld = LaunchDescription()
    ld.add_action(map_server_node)
    ld.add_action(nav2_lifecycle)
    ld.add_action(map_odom_tf_node)

    return ld
