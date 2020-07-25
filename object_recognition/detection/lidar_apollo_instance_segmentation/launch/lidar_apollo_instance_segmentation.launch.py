# Copyright 2020 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    """
    Launch lidar apollo instance segmentation node
    """
    n = []
    lidar_apollo_instance_segmentation_dir = get_package_share_directory('lidar_apollo_instance_segmentation')
    model = LaunchConfiguration(
        'model',
        default='128'
    )
    # Arguments
    n.append(DeclareLaunchArgument(
        'model',
        default_value='128',
        description='lidar_model'
    ))
    n.append(DeclareLaunchArgument(
        'prototxt_file',
        default_value=os.path.join(lidar_apollo_instance_segmentation_dir, 'data/vls-128.prototxt'),
        description='prototxt file for lidar apollo instance segmentation',
        condition=IfCondition(PythonExpression([model, '==128']))
    ))
    n.append(DeclareLaunchArgument(
        'caffemodel_file',
        default_value=os.path.join(lidar_apollo_instance_segmentation_dir, 'data/vls-128.caffemodel'),
        description='caffemodel file for lidar apollo instance segmentation',
        condition=IfCondition(PythonExpression([model, '==128']))
    ))
    n.append(DeclareLaunchArgument(
        'engine_file',
        default_value=os.path.join(lidar_apollo_instance_segmentation_dir, 'data/vls-128.engine'),
        description='engine file for lidar apollo instance segmentation',
        condition=IfCondition(PythonExpression([model, '==128']))
    ))
    n.append(DeclareLaunchArgument(
        'prototxt_file',
        default_value=os.path.join(lidar_apollo_instance_segmentation_dir, 'data/hdl-64.prototxt'),
        description='prototxt file for lidar apollo instance segmentation',
        condition=IfCondition(PythonExpression([model, '==64']))
    ))
    n.append(DeclareLaunchArgument(
        'caffemodel_file',
        default_value=os.path.join(lidar_apollo_instance_segmentation_dir, 'data/hdl-64.caffemodel'),
        description='caffemodel file for lidar apollo instance segmentation',
        condition=IfCondition(PythonExpression([model, '==64']))
    ))
    n.append(DeclareLaunchArgument(
        'engine_file',
        default_value=os.path.join(lidar_apollo_instance_segmentation_dir, 'data/hdl-64.engine'),
        description='engine file for lidar apollo instance segmentation',
        condition=IfCondition(PythonExpression([model, '==64']))
    ))
    n.append(DeclareLaunchArgument(
        'prototxt_file',
        default_value=os.path.join(lidar_apollo_instance_segmentation_dir, 'data/vlp-16.prototxt'),
        description='prototxt file for lidar apollo instance segmentation',
        condition=IfCondition(PythonExpression([model, '==16']))
    ))
    n.append(DeclareLaunchArgument(
        'caffemodel_file',
        default_value=os.path.join(lidar_apollo_instance_segmentation_dir, 'data/vlp-16.caffemodel'),
        description='caffemodel file for lidar apollo instance segmentation',
        condition=IfCondition(PythonExpression([model, '==16']))
    ))
    n.append(DeclareLaunchArgument(
        'engine_file',
        default_value=os.path.join(lidar_apollo_instance_segmentation_dir, 'data/vlp-16.engine'),
        description='engine file for lidar apollo instance segmentation',
        condition=IfCondition(PythonExpression([model, '==16']))
    ))
    n.append(DeclareLaunchArgument(
        'output/objects',
        default_value='labeled_clusters',
        description='output topic name'
    ))
    n.append(DeclareLaunchArgument(
        'target_frame',
        default_value='base_link',
        description='target frame'
    ))
    n.append(DeclareLaunchArgument(
        'z_offset',
        default_value='-2.0',
        description='z offset'
    ))
    # Nodes
    n.append(Node(
        package='lidar_apollo_instance_segmentation',
        node_executable='lidar_apollo_instance_segmentation_node_exe',
        node_namespace='',
        output='screen',
        parameters=[[lidar_apollo_instance_segmentation_dir, '/param/model_', LaunchConfiguration('model'), '.yaml'],
                    {'prototxt_file': LaunchConfiguration('prototxt_file'),
                     'caffemodel_file': LaunchConfiguration('caffemodel_file'),
                     'engine_file': LaunchConfiguration('engine_file'),
                     'target_frame': LaunchConfiguration('target_frame'),
                     'z_offset': LaunchConfiguration('z_offset'),}
                     ],
        remappings=[
            ("input/pointcloud", '/points_raw'),
            ("output/labeled_clusters", LaunchConfiguration('output/objects'))
        ]
    ))

    return LaunchDescription(n)
