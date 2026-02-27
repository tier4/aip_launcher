# Copyright 2025 TIER IV, Inc.
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

"""
front_lower の pointcloud_raw_ex に CropBox をかけて点群を抽出する launch。
LiDAR 座標（front_lower/lidar）で範囲指定。TF 不要で扱いやすい。

入力: /sensing/lidar/front_lower/pointcloud_raw_ex
出力: /sensing/lidar/front_lower/pointcloud_cropped

使い方:
  ros2 launch aip_x2_gen2_launch front_lower_cropbox_filter.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    pkg_share = get_package_share_directory("aip_x2_gen2_launch")
    param_path = os.path.join(pkg_share, "config", "front_lower_cropbox_filter.param.yaml")

    declare_param_file = DeclareLaunchArgument(
        "front_lower_cropbox_param_file",
        default_value=param_path,
        description="Path to front_lower cropbox filter param file",
    )

    param_file = ParameterFile(
        param_file=LaunchConfiguration("front_lower_cropbox_param_file"),
        allow_substs=True,
    )

    # CropBox: 入力は pointcloud_raw_ex（LiDAR 座標）。input_frame=front_lower/lidar で TF 変換なし
    cropbox_component = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter",
        namespace="sensing/lidar/front_lower",
        remappings=[
            ("input", "/sensing/lidar/front_lower/pointcloud_raw_ex"),
            ("output", "pointcloud_cropped"),
        ],
        parameters=[param_file],
    )

    container = ComposableNodeContainer(
        name="front_lower_cropbox_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[cropbox_component],
        output="screen",
    )

    return LaunchDescription([
        declare_param_file,
        container,
    ])
