#!/usr/bin/env python3
# Copyright 2025 TIER IV, Inc.
#
# front_lower の点群を「最新 TF」で map に変換して publish する。
# CropBox が点群の stamp で map→lidar を取ると過去外挿で落ちるため、
# このノードで先に map 変換しておき、CropBox には frame_id=map の点群を渡す。

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

# センサー系トピックと合わせる（pointcloud_raw_ex / CropBox は SensorDataQoS = BEST_EFFORT）
SENSOR_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def _quat_to_rotation_matrix(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) to 3x3 rotation matrix."""
    return np.array([
        [
            1 - 2 * (qy * qy + qz * qz),
            2 * (qx * qy - qz * qw),
            2 * (qx * qz + qy * qw),
        ],
        [
            2 * (qx * qy + qz * qw),
            1 - 2 * (qx * qx + qz * qz),
            2 * (qy * qz - qx * qw),
        ],
        [
            2 * (qx * qz - qy * qw),
            2 * (qy * qz + qx * qw),
            1 - 2 * (qx * qx + qy * qy),
        ],
    ])


def transform_point(trans, x: float, y: float, z: float):
    """TransformStamped で 1 点を変換。trans: target_frame <- source_frame."""
    t = trans.transform.translation
    r = trans.transform.rotation
    R = _quat_to_rotation_matrix(r.x, r.y, r.z, r.w)
    p = np.array([x, y, z])
    return (R @ p) + np.array([t.x, t.y, t.z])


class TransformPointcloudToMapNode(Node):
    def __init__(self):
        super().__init__("transform_pointcloud_to_map")
        self.declare_parameter("input_topic", "/sensing/lidar/front_lower/pointcloud_raw_ex")
        self.declare_parameter("output_topic", "/sensing/lidar/front_lower/pointcloud_in_map")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("use_fixed_transform", True)  # True: 最初に取得した TF を固定して揺れを抑える
        self._input_topic = self.get_parameter("input_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._target_frame = self.get_parameter("target_frame").value
        self._use_fixed_transform = self.get_parameter("use_fixed_transform").value
        self._fixed_trans = None  # use_fixed_transform 時にキャッシュ

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._sub = self.create_subscription(
            PointCloud2,
            self._input_topic,
            self._callback,
            SENSOR_QOS,
        )
        self._pub = self.create_publisher(PointCloud2, self._output_topic, SENSOR_QOS)

    def _callback(self, msg: PointCloud2):
        source_frame = msg.header.frame_id
        if source_frame == self._target_frame:
            self._pub.publish(msg)
            return

        # 固定 TF モード: 一度取得した transform を使い回して揺れを抑える
        if self._use_fixed_transform and self._fixed_trans is not None:
            trans = self._fixed_trans
        else:
            try:
                trans = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    source_frame,
                    rclpy.time.Time(),
                )
                if self._use_fixed_transform:
                    self._fixed_trans = trans
                    self.get_logger().info(
                        f"Fixed transform cached: {self._target_frame} <- {source_frame}"
                    )
            except TransformException as e:
                self.get_logger().warn(
                    f"Transform {self._target_frame} -> {source_frame} not available: {e}",
                    throttle_duration_sec=1.0,
                )
                return

        field_names = [f.name for f in msg.fields]
        points_out = []
        for p in pc2.read_points(msg, field_names=field_names, skip_nans=True):
            p_list = list(p)
            idx_x = field_names.index("x")
            idx_y = field_names.index("y")
            idx_z = field_names.index("z")
            nx, ny, nz = transform_point(trans, p_list[idx_x], p_list[idx_y], p_list[idx_z])
            p_list[idx_x], p_list[idx_y], p_list[idx_z] = nx, ny, nz
            points_out.append(tuple(p_list))

        if not points_out:
            return

        out_header = msg.header
        out_header.frame_id = self._target_frame
        out_msg = pc2.create_cloud(out_header, msg.fields, points_out)
        self._pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TransformPointcloudToMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
