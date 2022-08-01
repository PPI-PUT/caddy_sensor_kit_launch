# Copyright 2020 Tier IV, Inc. All rights reserved.
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
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def get_vehicle_info(vehicle_params):
    crop_params = {}
    crop_params["vehicle_length"] = vehicle_params["front_overhang"] + vehicle_params["wheel_base"] + vehicle_params["rear_overhang"]
    crop_params["vehicle_width"] = vehicle_params["wheel_tread"] + vehicle_params["left_overhang"] + vehicle_params["right_overhang"]
    crop_params["min_longitudinal_offset"] = -vehicle_params["rear_overhang"]
    crop_params["max_longitudinal_offset"] = vehicle_params["front_overhang"] + vehicle_params["wheel_base"]
    crop_params["min_lateral_offset"] = -(vehicle_params["wheel_tread"] / 2.0 + vehicle_params["right_overhang"])
    crop_params["max_lateral_offset"] = vehicle_params["wheel_tread"] / 2.0 + vehicle_params["left_overhang"]
    crop_params["min_height_offset"] = 0.3
    crop_params["max_height_offset"] = vehicle_params["vehicle_height"]
    return crop_params


def get_config(context, param):
    path = LaunchConfiguration(param).perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    lidar_front_params = get_config(context, 'lidar_front_params')
    lidar_rear_params = get_config(context, 'lidar_rear_params')
    vehicle_params = get_config(context, 'vehicle_params')
    vehicle_info = get_vehicle_info(vehicle_params)

    nodes = []

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
            name="concatenate_filter",
            namespace="lidars",
            remappings=[("output", "concatenated")],
            parameters=[
                {
                    "input_topics": ["/lidar_front/points", "/lidar_rear/points"],
                    "output_frame": LaunchConfiguration("output_frame"),
                    "approximate_sync": True,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            namespace="lidars",
            remappings=[
                ("input", "concatenated"),
                ("output", "cropped"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # mirror_info = get_vehicle_mirror_info(context)
    # cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    # cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    # cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    # cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    # cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    # cropbox_parameters["max_z"] = mirror_info["max_height_offset"]
    #
    # nodes.append(
    #     ComposableNode(
    #         package="pointcloud_preprocessor",
    #         plugin="pointcloud_preprocessor::CropBoxFilterComponent",
    #         name="crop_box_filter_mirror",
    #         remappings=[
    #             ("input", "self_cropped/pointcloud_ex"),
    #             ("output", "mirror_cropped/pointcloud_ex"),
    #         ],
    #         parameters=[cropbox_parameters],
    #         extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    #     )
    # )
    #
    # nodes.append(
    #     ComposableNode(
    #         package="pointcloud_preprocessor",
    #         plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
    #         name="distortion_corrector_node",
    #         remappings=[
    #             ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
    #             ("~/input/imu", "/sensing/imu/imu_data"),
    #             ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
    #             ("~/output/pointcloud", "rectified/pointcloud_ex"),
    #         ],
    #         extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    #     )
    # )
    #
    # nodes.append(
    #     ComposableNode(
    #         package="pointcloud_preprocessor",
    #         plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
    #         name="ring_outlier_filter",
    #         namespace="lidars",
    #         remappings=[
    #             ("input", "cropped"),
    #             ("output", "corrected"),
    #         ],
    #         extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    #     )
    # )
    #
    # # set container to run all required components in the same process
    # container = ComposableNodeContainer(
    #     # need unique name, otherwise all processes in same container and the node names then clash
    #     name="velodyne_node_container",
    #     namespace="pointcloud_preprocessor",
    #     package="rclcpp_components",
    #     executable=LaunchConfiguration("container_executable"),
    #     composable_node_descriptions=nodes,
    # )

    driver_component_front = ComposableNode(
        package="ros2_ouster",
        plugin="ros2_ouster::Driver",
        name="ouster_driver_front",
        namespace="lidar_front",
        parameters=[lidar_front_params],
        # extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    driver_component_rear = ComposableNode(
        package="ros2_ouster",
        plugin="ros2_ouster::Driver",
        name="ouster_driver_rear",
        namespace="lidar_rear",
        parameters=[lidar_rear_params],
        # extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
                        name='lidar_container',
                        namespace='lidars',
                        package='rclcpp_components',
                        executable='component_container',
                        composable_node_descriptions=nodes,
                        output='screen')

    # one way to add a ComposableNode conditional on a launch argument to a
    # container. The `ComposableNode` itself doesn't accept a condition
    loader = LoadComposableNodes(
        composable_node_descriptions=[driver_component_front, driver_component_rear],
        target_container=container,
        condition=launch.conditions.IfCondition(LaunchConfiguration("launch_driver")),
    )

    return [container, loader]


def generate_launch_description():
    share_dir = get_package_share_directory('caddy_sensor_kit_launch')
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("lidar_front_params", default_value=os.path.join(share_dir, 'config/lidar', 'lidar_front.param.yaml'),
                   description='FPath to the ROS2 parameters file to use.')
    add_launch_arg("lidar_rear_params", default_value=os.path.join(share_dir, 'config/lidar', 'lidar_rear.param.yaml'),
                   description='FPath to the ROS2 parameters file to use.')
    add_launch_arg("vehicle_params", default_value=os.path.join(share_dir, 'config/vehicle', 'vehicle_info.param.yaml'),
                   description='FPath to the ROS2 parameters file to use.')

    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg("base_frame", "base_link", "base frame id")
    # add_launch_arg("min_range", description="minimum view range")
    # add_launch_arg("max_range", description="maximum view range")
    # add_launch_arg("read_fast", "False")
    # add_launch_arg("read_once", "False")
    # add_launch_arg("repeat_delay", "0.0")
    # add_launch_arg("rpm", "600.0", "rotational frequency")
    # add_launch_arg("laserscan_ring", "-1")
    # add_launch_arg("laserscan_resolution", "0.007")
    # add_launch_arg("num_points_thresholds", "300")
    # add_launch_arg("invalid_intensity")
    # add_launch_arg("gps_time", "False")
    # add_launch_arg("view_direction", description="the center of lidar angle")
    # add_launch_arg("view_width", description="lidar angle: 0~6.28 [rad]")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for pcd process")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for pcd process")
    # add_launch_arg(
    #     "vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml"
    # )
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "True", "use ROS2 component container communication")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
