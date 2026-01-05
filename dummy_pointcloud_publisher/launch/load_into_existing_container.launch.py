import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    pkg_name = "dummy_pointcloud_publisher"
    default_param_path = os.path.join(
        get_package_share_directory(pkg_name),
        "config",
        "topic_info.param.yaml",
    )

    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=default_param_path,
        description="Path to ROS 2 parameter YAML",
    )

    # 例: /perception/perception_container など、既存コンテナのフル名を指定
    target_container_arg = DeclareLaunchArgument(
        "target_container",
        default_value="/pointcloud_container",
        description="Target container name",
    )

    param_file = ParameterFile(
        LaunchConfiguration("param_file"),
        allow_substs=True,
    )

    load = LoadComposableNodes(
        target_container=LaunchConfiguration("target_container"),
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin="dummy_pointcloud_publisher::DummyPointCloudPublisher",
                name="dummy_pointcloud_publisher",
                parameters=[param_file],
            )
        ],
    )

    return LaunchDescription([
        param_file_arg,
        target_container_arg,
        load,
    ])