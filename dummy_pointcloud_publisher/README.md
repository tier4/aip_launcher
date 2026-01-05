# dummy_pointcloud_publisher

YAMLで指定した**ダミーの `sensor_msgs/PointCloud2` トピック**を publish する ROS 2（Humble想定）パッケージです。
コンポーネント（Composable Node）として実装しており、**既存のコンテナへロード**して利用できます。

## 機能概要

- YAMLファイルで指定した **複数トピック**に対して `PointCloud2` を出力
- `QoS` は LiDAR 代替用途を想定し `SensorDataQoS` を使用
- 点群は基本 **空（width=0）** のダミー

---

## ROS パラメータ

| Parameter     | Type       | 内容                                                                                                         |
| ------------- | ---------- | ------------------------------------------------------------------------------------------------------------ |
| `topic_names` | `string[]` | publish するトピック名の配列（必須、1個以上）                                                                |
| `frame_ids`   | `string[]` | `header.frame_id` に設定する frame 名。サイズは **1 / topic_names と同数** を許可（1要素なら全トピック共通） |
| `rate_hz`     | `double`   | publish 周期 [Hz]（全トピック共通）                                                                          |

### YAML例（`config/topic_info.param.yaml`）

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

## 起動方法

### 1) ノード単体を起動（launch.xml）

`dummy_pointcloud_publisher.launch.xml` を起動すると、ノード単体でダミートピックを publish します。
（YAMLを読み込んでパラメータを渡す前提）

```bash
ros2 launch dummy_pointcloud_publisher dummy_pointcloud_publisher.launch.xml
```

---

### 2) 既存コンテナへロード（Python launch）

既に起動している `component_container`（デフォルト: `/pointcloud_container`）に対して、`dummy_pointcloud_publisher` を追加ロードします。

```bash
ros2 launch dummy_pointcloud_publisher load_into_existing_container.launch.py
```


---

### 3) テスト用：空コンテナを起動（Python launch）

ロード先コンテナが無い場合、テスト用に **空のコンポーネントコンテナ**だけを起動できます。

```bash
ros2 launch dummy_pointcloud_publisher empty_container.launch.py
```

別ターミナルで 2) のロードを実行して疎通確認します。

---
