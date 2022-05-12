# Copyright 2022 TIER IV, Inc.
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
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(*args, **kwargs):

    container = ComposableNodeContainer(
        name="image_diagnostics_container",
        namespace="image_diagnostics",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
    )
    image_diagnostics_component = ComposableNode(
        package="image_diagnostics",
        plugin="image_diagnostics::CameraPreprocessorNode",
        name="image_diagnostics_node",
        remappings=[
            ("input/compressed_image", "image_raw/compressed"),
            ("image_diag/raw_image", "image_diag/raw_image"),
        ],
    )
    image_diagnostics_loader = LoadComposableNodes(
        composable_node_descriptions=[image_diagnostics_component],
        target_container=container,
    )
    return [image_diagnostics_loader]


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=launch_setup)])
