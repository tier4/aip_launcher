# Copyright 2026 TIER IV, Inc.
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    container_name_arg = DeclareLaunchArgument(
        "container_name",
        default_value="pointcloud_container",
        description="Composable node container name",
    )

    # ComposableNodeContainer for MultiThreaded
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[],
        output="screen",
    )

    return LaunchDescription(
        [
            container_name_arg,
            container,
        ]
    )
