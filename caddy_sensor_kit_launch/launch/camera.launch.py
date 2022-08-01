# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#

import os
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
import yaml


def get_config(context, param):
    path = LaunchConfiguration(param).perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    camera_params = get_config(context, 'camera_params')
    flir_dir = get_package_share_directory('flir_spinnaker_ros2')
    config_dir = flir_dir + '/config/'

    camera_node = Node(package='flir_spinnaker_ros2',
                  executable='camera_driver_node',
                  output='screen',
                  name='blackfly_s',
                  namespace='camera',
                  parameters=[camera_params,
                              {'parameter_file': config_dir + 'blackfly_s.cfg',
                               'serial_number': [LaunchConfiguration('serial')]}],
                  remappings=[('~/control', '/camera/exposure_control/control'), ])

    comp_debayer_node = ComposableNode(
                        package='image_proc',
                        plugin='image_proc::DebayerNode',
                        name='debayer_node',
                        namespace='camera',
                        remappings=[("camera_info", "blackfly_s/camera_info"),
                                    ("image_raw", "blackfly_s/image_raw"),
                                    ("image_color", "blackfly_s/image_color"),
                                    ("image_color/compressed", "blackfly_s/image_color/compressed"),
                                    ("image_color/compressedDepth", "blackfly_s/image_color/compressedDepth"),
                                    ("image_color/theora", "blackfly_s/image_color/theora"),
                                    ("image_mono", "blackfly_s/image_mono"),
                                    ("image_mono/compressed", "blackfly_s/image_mono/compressed"),
                                    ("image_mono/compressedDepth", "blackfly_s/image_mono/compressedDepth"),
                                    ("image_mono/theora", "blackfly_s/image_mono/theora"),
                                    ]
    )

    debayer_container = ComposableNodeContainer(
                        name='image_proc_container',
                        namespace='camera',
                        package='rclcpp_components',
                        executable='component_container',
                        composable_node_descriptions=[comp_debayer_node],
                        output='screen')
    return [camera_node, debayer_container]

def generate_launch_description():
    launch_arguments = []
    share_dir = get_package_share_directory('caddy_sensor_kit_launch')

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            LaunchArg(name, default_value=default_value, description=description)
        )

    add_launch_arg("serial", "'21126305'", "serial_number")
    add_launch_arg("camera_params", default_value=os.path.join(share_dir, 'config/camera', 'blackfly_s.param.yaml'),
                   description='FPath to the ROS2 parameters file to use.')

    return LaunchDescription(
        launch_arguments
        + [OpaqueFunction(function=launch_setup)]
    )
