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

    # MultiThreaded のコンテナ（基本これでOK）
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[],  # ★何もロードしない
        output="screen",
    )

    return LaunchDescription([
        container_name_arg,
        container,
    ])
