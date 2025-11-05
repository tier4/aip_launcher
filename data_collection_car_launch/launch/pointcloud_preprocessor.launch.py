#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context, *args, **kwargs):
    concatenate_and_time_sync_node_param = ParameterFile(
        param_file=LaunchConfiguration("concatenate_and_time_sync_node_param_path").perform(context),
        allow_substs=True,
    )

    concat_component = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_data",
        remappings=[
            ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
            ("output", "concatenated/pointcloud"),
            ("output_info", "concatenated/pointcloud_info"),
        ],
        parameters=[concatenate_and_time_sync_node_param],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    return [
        LoadComposableNodes(
            composable_node_descriptions=[concat_component],
            target_container=LaunchConfiguration("pointcloud_container_name"),
            condition=IfCondition(LaunchConfiguration("use_concat_filter")),
        )
    ]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    share_dir = get_package_share_directory("data_collection_car_launch")

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_concat_filter", "True")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")
    add_launch_arg(
        "concatenate_and_time_sync_node_param_path",
        os.path.join(share_dir, "config", "concatenate_and_time_sync_node.param.yaml"),
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
