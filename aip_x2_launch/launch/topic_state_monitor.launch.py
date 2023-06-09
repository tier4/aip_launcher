# Copyright 2021 Tier IV, Inc. All rights reserved.
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

from launch.launch_description import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    topic_state_monitor_gnss = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_gnss",
        parameters=[
            {
                "topic": "/sensing/gnss/pose",
                "topic_type": "geometry_msgs/msg/PoseStamped",
                "best_effort": False,
                "diag_name": "septentrio_topic_status",
                "warn_rate": 2.5,
                "error_rate": 0.5,
                "timeout": 5.0,
                "window_size": 10,
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="topic_state_monitor_container",
        namespace="topic_state_monitor",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            topic_state_monitor_gnss,
        ],
        output="screen",
    )

    return LaunchDescription([container])
