# Copyright 2024 Tier IV, Inc. All rights reserved.
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

from launch import LaunchDescription
from launch_ros.actions import Node


def make_topic_state_monitor_node(name, topic, topic_type, diag_name, warn_rate, error_rate):
    # Uses TypedTopicStateMonitorNode (agnocast::Node) with compile-time type registry
    # instead of rclcpp::GenericSubscription. See typed_topic_state_monitor_core.hpp.
    return Node(
        package="autoware_topic_state_monitor",
        executable="typed_topic_state_monitor_agnocast_node",
        name=name,
        parameters=[
            {
                "topic": topic,
                "topic_type": topic_type,
                "best_effort": True,
                "diag_name": diag_name,
                "warn_rate": warn_rate,
                "error_rate": error_rate,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        output="screen",
        additional_env={
            "LD_PRELOAD": "libagnocast_heaphook.so:"
            + os.environ.get("LD_PRELOAD", ""),
        },
    )


def generate_launch_description():
    nodes = [
        # GNSS topic monitor
        make_topic_state_monitor_node(
            name="topic_state_monitor_gnss_pose",
            topic="/sensing/gnss/pose",
            topic_type="geometry_msgs/msg/PoseStamped",
            diag_name="gnss_topic_status",
            warn_rate=2.5,
            error_rate=0.5,
        ),
        # IMU topic monitor
        make_topic_state_monitor_node(
            name="topic_state_monitor_imu_data",
            topic="/sensing/imu/imu_data",
            topic_type="sensor_msgs/msg/Imu",
            diag_name="imu_topic_status",
            warn_rate=5.0,
            error_rate=1.0,
        ),
        # Radar topic monitors
        make_topic_state_monitor_node(
            name="topic_state_monitor_radar_front_center",
            topic="/sensing/radar/front_center/nebula_packets",
            topic_type="nebula_msgs/msg/NebulaPackets",
            diag_name="radar_front_center_topic_status",
            warn_rate=20.0,
            error_rate=5.0,
        ),
        make_topic_state_monitor_node(
            name="topic_state_monitor_radar_front_left",
            topic="/sensing/radar/front_left/nebula_packets",
            topic_type="nebula_msgs/msg/NebulaPackets",
            diag_name="radar_front_left_topic_status",
            warn_rate=20.0,
            error_rate=5.0,
        ),
        make_topic_state_monitor_node(
            name="topic_state_monitor_radar_front_right",
            topic="/sensing/radar/front_right/nebula_packets",
            topic_type="nebula_msgs/msg/NebulaPackets",
            diag_name="radar_front_right_topic_status",
            warn_rate=20.0,
            error_rate=5.0,
        ),
        make_topic_state_monitor_node(
            name="topic_state_monitor_radar_rear_center",
            topic="/sensing/radar/rear_center/nebula_packets",
            topic_type="nebula_msgs/msg/NebulaPackets",
            diag_name="radar_rear_center_topic_status",
            warn_rate=20.0,
            error_rate=5.0,
        ),
        make_topic_state_monitor_node(
            name="topic_state_monitor_radar_rear_left",
            topic="/sensing/radar/rear_left/nebula_packets",
            topic_type="nebula_msgs/msg/NebulaPackets",
            diag_name="radar_rear_left_topic_status",
            warn_rate=20.0,
            error_rate=5.0,
        ),
        make_topic_state_monitor_node(
            name="topic_state_monitor_radar_rear_right",
            topic="/sensing/radar/rear_right/nebula_packets",
            topic_type="nebula_msgs/msg/NebulaPackets",
            diag_name="radar_rear_right_topic_status",
            warn_rate=20.0,
            error_rate=5.0,
        ),
    ]

    return LaunchDescription(nodes)
