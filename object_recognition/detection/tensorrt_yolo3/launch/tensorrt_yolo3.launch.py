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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch tensorrt yolo3 node
    """

    tensorrt_yolo3_dir = get_package_share_directory('tensorrt_yolo3')
    # Arguments
    prototxt_file = DeclareLaunchArgument(
        'prototxt_file',
        default_value=os.path.join(tensorrt_yolo3_dir, 'data/yolov3_416_trt.prototxt'),
        description='prototxt file for tensorrt yolo3'
    )

    caffemodel_file = DeclareLaunchArgument(
        'caffemodel_file',
        default_value=os.path.join(tensorrt_yolo3_dir, 'data/yolov3_416.caffemodel'),
        description='caffemodel file for tensorrt yolo3'
    )

    input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/image_raw',
        description='input topic name'
    )

    output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='rois',
        description='output topic name'
    )

    # Nodes
    tensorrt_yolo3 = Node(
        package='tensorrt_yolo3',
        node_executable='tensorrt_yolo3_node_exe',
        node_name='tensorrt_yolo3_node',
        node_namespace='',
        output='screen',
        parameters=[{'prototxt_file': LaunchConfiguration('prototxt_file'),
                     'caffemodel_file': LaunchConfiguration('caffemodel_file'), }],
        remappings=[
            ("in_image_base_topic", LaunchConfiguration('input_topic')),
            ("out_image_base_topic", [LaunchConfiguration('output_topic'), '/debug/image']),
            ("out_objects_topic", LaunchConfiguration('output_topic'))
        ]
    )

    return LaunchDescription([
        prototxt_file,
        caffemodel_file,
        input_topic,
        output_topic,
        tensorrt_yolo3
    ])
