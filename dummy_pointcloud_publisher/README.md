# dummy_pointcloud_publisher

This is a ROS 2 (Humble assumed) package that **publishes dummy `sensor_msgs/PointCloud2` topics** specified in a YAML file.
It is implemented as a **Composable Node (component)**, so you can **load it into an existing container** and use it there.

## Overview

- Publishes `PointCloud2` to **multiple topics** specified in a YAML file
- Uses `SensorDataQoS`, assuming usage as a LiDAR substitute
- The point cloud is basically a dummy with **no points (width=0)**

---

## ROS Parameters

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

## Verification / Expected Output (How to Run)

This section shows **example command outputs** for verifying that the package is working.

> NOTE: The outputs below are placeholders. Please replace them with the actual logs from your environment.

---

### 1) Run as a standalone node (launch.xml)

#### 1.1 Launch command

```bash
ros2 launch dummy_pointcloud_publisher dummy_pointcloud_publisher.launch.xml
```

##### Terminal output

```text
[INFO] [dummy_pointcloud_publisher_node-1]: process started with pid [130536]
[dummy_pointcloud_publisher_node-1] [INFO] [1768444166.031710974] [dummy_pointcloud_publisher]: Stream[0]: topic=/sensing/lidar/front_left/pointcloud_before_sync frame_id=hesai_front_left
[dummy_pointcloud_publisher_node-1] [INFO] [1768444166.032115272] [dummy_pointcloud_publisher]: Stream[1]: topic=/sensing/lidar/front_right/pointcloud_before_sync frame_id=hesai_front_right
[dummy_pointcloud_publisher_node-1] [INFO] [1768444166.032376408] [dummy_pointcloud_publisher]: Stream[2]: topic=/sensing/lidar/side_left/pointcloud_before_sync frame_id=hesai_side_left
[dummy_pointcloud_publisher_node-1] [INFO] [1768444166.032618582] [dummy_pointcloud_publisher]: Stream[3]: topic=/sensing/lidar/side_right/pointcloud_before_sync frame_id=hesai_side_right
[dummy_pointcloud_publisher_node-1] [INFO] [1768444166.032858725] [dummy_pointcloud_publisher]: Stream[4]: topic=/sensing/lidar/rear/pointcloud_before_sync frame_id=hesai_rear
[dummy_pointcloud_publisher_node-1] [INFO] [1768444166.033120525] [dummy_pointcloud_publisher]: Started. rate_hz=10.000 streams=5
```

#### 1.2 Check published topics

```bash
ros2 topic list | grep pointcloud
```

##### Terminal output
```text
/sensing/lidar/front_left/pointcloud_before_sync
/sensing/lidar/front_right/pointcloud_before_sync
/sensing/lidar/rear/pointcloud_before_sync
/sensing/lidar/side_left/pointcloud_before_sync
/sensing/lidar/side_right/pointcloud_before_sync
```

#### 1.3 Echo one of the dummy PointCloud2 topics

```bash
ros2 topic echo /sensing/lidar/front_left/pointcloud_before_sync --once
```

##### Terminal output
```text
header:
  stamp:
    sec: 1768445041
    nanosec: 241654439
  frame_id: hesai_front_left
height: 1
width: 0
fields:
- name: x
  offset: 0
  datatype: 7
  count: 1
- name: y
  offset: 4
  datatype: 7
  count: 1
- name: z
  offset: 8
  datatype: 7
  count: 1
- name: intensity
  offset: 12
  datatype: 2
  count: 1
- name: return_type
  offset: 13
  datatype: 2
  count: 1
- name: channel
  offset: 14
  datatype: 4
  count: 1
is_bigendian: false
point_step: 16
row_step: 0
data: []
is_dense: true
```

---

### 2) Load into a container (Python launch)

This method verifies that the component can be loaded into a container, which is useful when integrating with existing Autoware containers.

#### 2.1 Start an empty component container

```bash
ros2 launch dummy_pointcloud_publisher empty_container.launch.py
```

##### Terminal output (empty_container terminal)

```text
[INFO] [component_container_mt-1]: process started with pid [145271]
```

#### 2.2 Load `dummy_pointcloud_publisher` into the container (in another terminal)

```bash
ros2 launch dummy_pointcloud_publisher load_into_existing_container.launch.py \
  target_container:=/pointcloud_container
```

##### Terminal output (load_into_existing_container terminal)

```text
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/dummy_pointcloud_publisher' in container '/pointcloud_container'
```

##### Terminal output (empty_container terminal)

```text
[component_container_mt-1] [INFO] [1768445442.079591539] [pointcloud_container]: Load Library: /home/takerukimura/ros2_ws/install/dummy_pointcloud_publisher/lib/libdummy_pointcloud_publisher_component.so
[component_container_mt-1] [INFO] [1768445442.080939496] [pointcloud_container]: Found class: rclcpp_components::NodeFactoryTemplate<dummy_pointcloud_publisher::DummyPointCloudPublisher>
[component_container_mt-1] [INFO] [1768445442.080993777] [pointcloud_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<dummy_pointcloud_publisher::DummyPointCloudPublisher>
[component_container_mt-1] [INFO] [1768445442.088239869] [dummy_pointcloud_publisher]: Stream[0]: topic=/sensing/lidar/front_left/pointcloud_before_sync frame_id=hesai_front_left
[component_container_mt-1] [INFO] [1768445442.089611600] [dummy_pointcloud_publisher]: Stream[1]: topic=/sensing/lidar/front_right/pointcloud_before_sync frame_id=hesai_front_right
[component_container_mt-1] [INFO] [1768445442.090607990] [dummy_pointcloud_publisher]: Stream[2]: topic=/sensing/lidar/side_left/pointcloud_before_sync frame_id=hesai_side_left
[component_container_mt-1] [INFO] [1768445442.093602151] [dummy_pointcloud_publisher]: Stream[3]: topic=/sensing/lidar/side_right/pointcloud_before_sync frame_id=hesai_side_right
[component_container_mt-1] [INFO] [1768445442.094714725] [dummy_pointcloud_publisher]: Stream[4]: topic=/sensing/lidar/rear/pointcloud_before_sync frame_id=hesai_rear
[component_container_mt-1] [INFO] [1768445442.095618427] [dummy_pointcloud_publisher]: Started. rate_hz=10.000 streams=5
```

#### 2.3 Confirm the node is loaded

```bash
ros2 component list /pointcloud_container
```

##### Terminal output

```text
1  /dummy_pointcloud_publisher
```

#### 2.4 Confirm topics are published

```bash
ros2 topic list | grep pointcloud
```

##### Terminal output

```text
/sensing/lidar/front_left/pointcloud_before_sync
/sensing/lidar/front_right/pointcloud_before_sync
/sensing/lidar/rear/pointcloud_before_sync
/sensing/lidar/side_left/pointcloud_before_sync
/sensing/lidar/side_right/pointcloud_before_sync
```

#### 2.5 Echo one of the dummy PointCloud2 topics

```bash
ros2 topic echo /sensing/lidar/front_left/pointcloud_before_sync --once
```

##### Terminal output

```text
header:
  stamp:
    sec: 1768445751
    nanosec: 498563000
  frame_id: hesai_front_left
height: 1
width: 0
fields:
- name: x
  offset: 0
  datatype: 7
  count: 1
- name: y
  offset: 4
  datatype: 7
  count: 1
- name: z
  offset: 8
  datatype: 7
  count: 1
- name: intensity
  offset: 12
  datatype: 2
  count: 1
- name: return_type
  offset: 13
  datatype: 2
  count: 1
- name: channel
  offset: 14
  datatype: 4
  count: 1
is_bigendian: false
point_step: 16
row_step: 0
data: []
is_dense: true
```
