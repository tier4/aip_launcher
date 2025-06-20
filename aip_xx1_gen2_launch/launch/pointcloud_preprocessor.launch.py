# Copyright 2024 TIER IV, Inc. All rights reserved.
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
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def erase_rear_lidar_entry_depending_on_vehicle_id(config: dict, vehicle_id: str) -> dict:
    # Only NO. 8 vehicle does not have a rear lidar, so we erase the rear lidar entry.
    if vehicle_id != "8":
        return config

    # Acquire the index of the rear lidar entry
    rear_lidar_index = config["input_topics"].index("/sensing/lidar/rear/pointcloud_before_sync")

    # Remove all items related to the rear lidar
    config["input_topics"].pop(rear_lidar_index)
    config["matching_strategy"]["lidar_timestamp_offsets"].pop(rear_lidar_index)
    config["matching_strategy"]["lidar_timestamp_noise_window"].pop(rear_lidar_index)

    return config


def launch_setup(context, *args, **kwargs):
    # Load concatenate node parameters as YAML
    with open(
        LaunchConfiguration("concatenate_and_time_sync_node_param_path").perform(context), "r"
    ) as f:
        concatenate_and_time_sync_node_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # Remove the rear lidar entry from the parameter file if the vehicle does not have a rear lidar
    concatenate_and_time_sync_node_param = erase_rear_lidar_entry_depending_on_vehicle_id(
        concatenate_and_time_sync_node_param, LaunchConfiguration("vehicle_id").perform(context)
    )

    # set concat filter as a component
    concat_component = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_data",
        remappings=[
            ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
            ("output", "concatenated/pointcloud"),
        ],
        parameters=[concatenate_and_time_sync_node_param],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # load concat or passthrough filter
    concat_loader = LoadComposableNodes(
        composable_node_descriptions=[concat_component],
        target_container=LaunchConfiguration("pointcloud_container_name"),
        condition=IfCondition(LaunchConfiguration("use_concat_filter")),
    )

    return [concat_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    aip_xx1_gen2_launch_share_dir = get_package_share_directory("aip_xx1_gen2_launch")

    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")
    add_launch_arg("individual_container_name", "concatenate_container")
    add_launch_arg(
        "vehicle_id",
        default_value=EnvironmentVariable("VEHICLE_ID", default_value="default"),
    )
    add_launch_arg(
        "concatenate_and_time_sync_node_param_path",
        os.path.join(
            aip_xx1_gen2_launch_share_dir,
            "config",
            "concatenate_and_time_sync_node.param.yaml",
        ),
    )

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
