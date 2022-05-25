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

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def get_dual_return_filter_info(context):
    path = LaunchConfiguration("dual_return_filter_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def get_vehicle_info(context):
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = context.launch_configurations.get("global_params", {})
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    driver_component = ComposableNode(
        package="pandar_driver",
        plugin="pandar_driver::PandarDriver",
        name="pandar_driver",
        parameters=[
            {
                **create_parameter_dict(
                    "pcap", "device_ip", "lidar_port", "gps_port", "scan_phase", "model", "frame_id"
                )
            }
        ],
    )

    pointcloud_component = ComposableNode(
        package="pandar_pointcloud",
        plugin="pandar_pointcloud::PandarCloud",
        name="pandar_cloud",
        parameters=[
            {
                **create_parameter_dict(
                    "model",
                    "scan_phase",
                    "angle_range",
                    "distance_range",
                    "device_ip",
                    "calibration",
                    "return_mode",
                )
            }
        ],
        remappings=[("pandar_points", "pointcloud_raw"), ("pandar_points_ex", "pointcloud_raw_ex")],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    dual_return_filter_info = get_dual_return_filter_info(context)
    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"] - 0.15
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"] + 0.15
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    self_crop_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter_self",
        remappings=[
            ("input", "pointcloud_raw_ex"),
            ("output", "self_cropped/pointcloud_ex"),
        ],
        parameters=[cropbox_parameters],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    mirror_info = get_vehicle_mirror_info(context)
    right = mirror_info["right"]
    cropbox_parameters.update(
        min_x=right["min_longitudinal_offset"],
        max_x=right["max_longitudinal_offset"],
        min_y=right["min_lateral_offset"],
        max_y=right["max_lateral_offset"],
        min_z=right["min_height_offset"],
        max_z=right["max_height_offset"],
    )

    right_mirror_crop_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter_mirror_right",
        remappings=[
            ("input", "self_cropped/pointcloud_ex"),
            ("output", "right_mirror_cropped/pointcloud_ex"),
        ],
        parameters=[cropbox_parameters],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    left = mirror_info["left"]
    cropbox_parameters.update(
        min_x=left["min_longitudinal_offset"],
        max_x=left["max_longitudinal_offset"],
        min_y=left["min_lateral_offset"],
        max_y=left["max_lateral_offset"],
        min_z=left["min_height_offset"],
        max_z=left["max_height_offset"],
    )

    left_mirror_crop_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter_mirror_left",
        remappings=[
            ("input", "right_mirror_cropped/pointcloud_ex"),
            ("output", "mirror_cropped/pointcloud_ex"),
        ],
        parameters=[cropbox_parameters],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    undistort_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
        name="distortion_corrector_node",
        remappings=[
            ("~/input/velocity_report", "/vehicle/status/velocity_status"),
            ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
            ("~/output/pointcloud", "rectified/pointcloud_ex"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    ring_outlier_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
        name="ring_outlier_filter",
        remappings=[
            ("input", "rectified/pointcloud_ex"),
            ("output", "outlier_filtered/pointcloud"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    dual_return_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::DualReturnOutlierFilterComponent",
        name="dual_return_filter",
        remappings=[
            ("input", "rectified/pointcloud_ex"),
            ("output", "outlier_filtered/pointcloud"),
        ],
        parameters=[
            {
                "vertical_bins": LaunchConfiguration("vertical_bins"),
                "min_azimuth_deg": LaunchConfiguration("min_azimuth_deg"),
                "max_azimuth_deg": LaunchConfiguration("max_azimuth_deg"),
            }
        ]
        + [dual_return_filter_info],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    blockage_diag_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::BlockageDiagComponent",
        name="blockage_return_diag",
        remappings=[
            ("input", "pointcloud_raw_ex"),
            ("output", "blockage_diag/pointcloud"),
        ],
        parameters=[
            {
                "angle_range": LaunchConfiguration("angle_range"),
                "horizontal_ring_id": LaunchConfiguration("horizontal_ring_id"),
                "blockage_ratio_threshold": LaunchConfiguration("blockage_ratio_threshold"),
                "vertical_bins": LaunchConfiguration("vertical_bins"),
                "model": LaunchConfiguration("model"),
                "blockage_count_threshold": LaunchConfiguration("blockage_count_threshold"),
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
        name="pandar_node_container",
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            pointcloud_component,
            self_crop_component,
            right_mirror_crop_component,
            left_mirror_crop_component,
            undistort_component,
        ],
    )

    driver_loader = LoadComposableNodes(
        composable_node_descriptions=[driver_component],
        target_container=container,
        condition=launch.conditions.IfCondition(LaunchConfiguration("launch_driver")),
    )

    ring_outlier_filter_loader = LoadComposableNodes(
        composable_node_descriptions=[ring_outlier_filter_component],
        target_container=container,
        condition=LaunchConfigurationNotEquals("return_mode", "Dual"),
    )

    dual_return_filter_loader = LoadComposableNodes(
        composable_node_descriptions=[dual_return_filter_component],
        target_container=container,
        condition=LaunchConfigurationEquals("return_mode", "Dual"),
    )

    blockage_diag_loader = LoadComposableNodes(
        composable_node_descriptions=[blockage_diag_component],
        target_container=container,
    )

    return [
        container,
        driver_loader,
        ring_outlier_filter_loader,
        dual_return_filter_loader,
        blockage_diag_loader,
    ]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("model")
    add_launch_arg("launch_driver", "true")
    add_launch_arg("calibration", "")
    add_launch_arg("device_ip", "192.168.1.201")
    add_launch_arg("scan_phase", "0.0")
    add_launch_arg("angle_range", "[270.0, 90.0]")
    add_launch_arg("distance_range", "[0.1, 200.0]")
    add_launch_arg("return_mode", "Dual")
    add_launch_arg("base_frame", "base_link")
    add_launch_arg("container_name", "pandar_composable_node_container")
    add_launch_arg("pcap", "")
    add_launch_arg("lidar_port", "2321")
    add_launch_arg("gps_port", "10121")
    add_launch_arg("frame_id", "pandar")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"))
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"))
    add_launch_arg("dual_return_filter_param_file")
    add_launch_arg("vehicle_mirror_param_file")
    add_launch_arg("use_multithread", "true")
    add_launch_arg("use_intra_process", "true")
    add_launch_arg("vertical_bins", "40")
    add_launch_arg("blockage_ratio_threshold", "0.1")
    add_launch_arg("horizontal_ring_id", "12")
    add_launch_arg("blockage_count_threshold", "50")

    add_launch_arg("min_azimuth_deg", "135.0")
    add_launch_arg("max_azimuth_deg", "225.0")
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
