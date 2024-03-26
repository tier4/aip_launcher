from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # GNSS topic monitor
    gnss_topic_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_gnss_pose",
        parameters=[
            {
                "topic": "/sensing/gnss/pose",
                "topic_type": "geometry_msgs/msg/PoseStamped",
                "best_effort": True,
                "diag_name": "gnss_topic_status",
                "warn_rate": 2.5,
                "error_rate": 0.5,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # IMU topic monitor
    imu_topic_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_imu_data",
        parameters=[
            {
                "topic": "/sensing/imu/imu_data",
                "topic_type": "sensor_msgs/msg/Imu",
                "best_effort": True,
                "diag_name": "imu_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Radar topic monitors
    radar_front_center_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_front_center",
        parameters=[
            {
                "topic": "/sensing/radar/front_center/objects_raw",
                "topic_type": "radar_msgs/msg/RadarTracks",
                "best_effort": True,
                "diag_name": "radar_front_center_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    radar_front_left_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_front_left",
        parameters=[
            {
                "topic": "/sensing/radar/front_left/objects_raw",
                "topic_type": "radar_msgs/msg/RadarTracks",
                "best_effort": True,
                "diag_name": "radar_front_left_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    radar_front_right_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_front_right",
        parameters=[
            {
                "topic": "/sensing/radar/front_right/objects_raw",
                "topic_type": "radar_msgs/msg/RadarTracks",
                "best_effort": True,
                "diag_name": "radar_front_right_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    radar_rear_center_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_rear_center",
        parameters=[
            {
                "topic": "/sensing/radar/rear_center/objects_raw",
                "topic_type": "radar_msgs/msg/RadarTracks",
                "best_effort": True,
                "diag_name": "radar_rear_center_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    radar_rear_left_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_rear_left",
        parameters=[
            {
                "topic": "/sensing/radar/rear_left/objects_raw",
                "topic_type": "radar_msgs/msg/RadarTracks",
                "best_effort": True,
                "diag_name": "radar_rear_left_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    radar_rear_right_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_radar_rear_right",
        parameters=[
            {
                "topic": "/sensing/radar/rear_right/objects_raw",
                "topic_type": "radar_msgs/msg/RadarTracks",
                "best_effort": True,
                "diag_name": "radar_rear_right_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Camera topic monitors
    camera0_topic_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_camera0",
        parameters=[
            {
                "topic": "/sensing/camera/camera0/camera_info",
                "topic_type": "sensor_msgs/msg/CameraInfo",
                "best_effort": True,
                "diag_name": "camera0_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    camera1_topic_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_camera1",
        parameters=[
            {
                "topic": "/sensing/camera/camera1/camera_info",
                "topic_type": "sensor_msgs/msg/CameraInfo",
                "best_effort": True,
                "diag_name": "camera1_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    camera2_topic_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_camera2",
        parameters=[
            {
                "topic": "/sensing/camera/camera2/camera_info",
                "topic_type": "sensor_msgs/msg/CameraInfo",
                "best_effort": True,
                "diag_name": "camera2_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    camera3_topic_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_camera3",
        parameters=[
            {
                "topic": "/sensing/camera/camera3/camera_info",
                "topic_type": "sensor_msgs/msg/CameraInfo",
                "best_effort": True,
                "diag_name": "camera3_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    camera4_topic_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_camera4",
        parameters=[
            {
                "topic": "/sensing/camera/camera4/camera_info",
                "topic_type": "sensor_msgs/msg/CameraInfo",
                "best_effort": True,
                "diag_name": "camera4_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    camera5_topic_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_camera5",
        parameters=[
            {
                "topic": "/sensing/camera/camera5/camera_info",
                "topic_type": "sensor_msgs/msg/CameraInfo",
                "best_effort": True,
                "diag_name": "camera5_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    camera6_topic_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_camera6",
        parameters=[
            {
                "topic": "/sensing/camera/camera6/camera_info",
                "topic_type": "sensor_msgs/msg/CameraInfo",
                "best_effort": True,
                "diag_name": "camera6_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    camera7_topic_monitor = ComposableNode(
        package="topic_state_monitor",
        plugin="topic_state_monitor::TopicStateMonitorNode",
        name="topic_state_monitor_camera7",
        parameters=[
            {
                "topic": "/sensing/camera/camera7/camera_info",
                "topic_type": "sensor_msgs/msg/CameraInfo",
                "best_effort": True,
                "diag_name": "camera7_topic_status",
                "warn_rate": 5.0,
                "error_rate": 1.0,
                "timeout": 5.0,
                "window_size": 10,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # ComposableNodeContainer to run all ComposableNodes
    container = ComposableNodeContainer(
        name="topic_state_monitor_container",
        namespace="topic_state_monitor",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            gnss_topic_monitor,
            imu_topic_monitor,
            radar_front_center_monitor,
            radar_front_left_monitor,
            radar_front_right_monitor,
            radar_rear_center_monitor,
            radar_rear_left_monitor,
            radar_rear_right_monitor,
            camera0_topic_monitor,
            camera1_topic_monitor,
            camera2_topic_monitor,
            camera3_topic_monitor,
            camera4_topic_monitor,
            camera5_topic_monitor,
            camera6_topic_monitor,
            camera7_topic_monitor,
        ],
        output="screen",
    )

    return LaunchDescription([container])
