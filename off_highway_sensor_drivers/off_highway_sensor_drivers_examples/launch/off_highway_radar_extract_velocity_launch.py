# Copyright 2024 Robert Bosch GmbH and its subsidiaries
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('off_highway_radar'),
        'config',
        'sender_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('params',
                              default_value=params,
                              description='Parameters for sender'),
        ComposableNodeContainer(
            package='rclcpp_components',
            name='off_highway_radar_container',
            namespace='',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='off_highway_sensor_drivers_examples',
                    plugin='off_highway_sensor_drivers_examples::ExtractVelocity',
                    name='off_highway_radar_transform_velocity',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='off_highway_radar',
                    plugin='off_highway_radar::Sender',
                    name='off_highway_radar_sender',
                    parameters=[LaunchConfiguration('params')],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ]
        )
    ])
