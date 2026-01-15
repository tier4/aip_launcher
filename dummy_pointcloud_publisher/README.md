# dummy_pointcloud_publisher

This is a ROS 2 (Humble assumed) package that **publishes dummy `sensor_msgs/PointCloud2` topics** specified in a YAML file.
It is implemented as a **Composable Node (component)**, so you can **load it into an existing container** and use it there.

## Overview

- Publishes `PointCloud2` to **multiple topics** specified in a YAML file
- Uses `SensorDataQoS`, assuming usage as a LiDAR substitute
- The point cloud is basically a dummy with **no points (width=0)**

---

## ROS パラメータ

| Parameter     | Type       | Description                                                                                                                       |
| ------------- | ---------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `topic_names` | `string[]` | Array of topic names to publish (required, at least one)                                                                          |
| `frame_ids`   | `string[]` | Frame names to set in `header.frame_id`. Size can be **1 or the same as `topic_names`** (if 1 element, it is used for all topics) |
| `rate_hz`     | `double`   | Publish rate in [Hz] (shared for all topics)                                                                                      |

### YAML examples（`config/topic_info.param.yaml`）

```yaml
/**:
  ros__parameters:
    topic_names:
      - /sensing/lidar/front_left/pointcloud_before_sync
      - /sensing/lidar/front_right/pointcloud_before_sync
      - /sensing/lidar/side_left/pointcloud_before_sync
      - /sensing/lidar/side_right/pointcloud_before_sync
      - /sensing/lidar/rear/pointcloud_before_sync
    frame_ids: [base_link]
    rate_hz: 10.0
```

```yaml
/**:
  ros__parameters:
    topic_names:
      - /sensing/lidar/front_left/pointcloud_before_sync
      - /sensing/lidar/front_right/pointcloud_before_sync
      - /sensing/lidar/side_left/pointcloud_before_sync
      - /sensing/lidar/side_right/pointcloud_before_sync
      - /sensing/lidar/rear/pointcloud_before_sync
    frame_ids:
      - hesai_front_left
      - hesai_front_right
      - hesai_side_left
      - hesai_side_right
      - hesai_rear
    rate_hz: 10.0
```

---

## How to Run

### 1) Run as a standalone node (launch.xml)

Launching `dummy_pointcloud_publisher.launch.xml` will publish the dummy topics as a standalone node.
(It assumes parameters are provided by loading the YAML.)

```bash
ros2 launch dummy_pointcloud_publisher dummy_pointcloud_publisher.launch.xml
```

---

### 2) Load into an existing container (Python launch)

Adds `dummy_pointcloud_publisher` into an already running `component_container` (default: `/pointcloud_container`).

```bash
ros2 launch dummy_pointcloud_publisher load_into_existing_container.launch.py
  target_container:=/pointcloud_container
```

---

- `target_container`: destination container name

### 3) For testing: launch an empty container (Python launch)

If there is no destination container, you can launch an **empty component container** for testing.

```bash
ros2 launch dummy_pointcloud_publisher empty_container.launch.py
```

Then run step (2) in another terminal to verify communication.

---
