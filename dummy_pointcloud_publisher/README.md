# dummy_pointcloud_publisher

YAMLで指定した設定に基づき、**ダミーの `sensor_msgs/PointCloud2` トピック**を publish する ROS 2（Humble想定）パッケージです。
コンポーネント（Composable Node）として実装しており、**既存のコンテナへロード**して利用できます。

## 機能概要

- YAMLファイルで指定した **複数トピック**に対して `PointCloud2` を出力
- `QoS` は LiDAR 代替用途を想定し `SensorDataQoS` を使用
- 点群は基本 **空（width=0）** のダミー（下流の疎通確認用途）

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
      - /dummy/lidar/front/pointcloud
      - /dummy/lidar/rear/pointcloud
    frame_ids: [lidar_dummy]
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

> ※ launch.xml 側の実装によっては、param ファイルのパス指定方法が異なる場合があります。必要なら launch.xml 内の `<param from="...">` 等を確認してください。

---

### 2) 既存コンテナへロード（Python launch）

既に起動している `component_container`（例：Autoware の node_container）に対して、Composable Node を追加ロードします。

```bash
ros2 launch dummy_pointcloud_publisher load_dummy_into_existing_container.launch.py \
  target_container:=/dummy_container \
  param_file:=/path/to/topic_info.param.yaml
```

- `target_container`：ロード先コンテナ名（先頭 `/` 付き推奨）
- `param_file`：パラメータ YAML のパス

---

### 3) テスト用：空コンテナを起動（Python launch）

ロード先コンテナが無い場合、テスト用に **空のコンポーネントコンテナ**だけを起動できます。

```bash
ros2 launch dummy_pointcloud_publisher empty_container.launch.py container_name:=dummy_container
```

別ターミナルで 2) のロードを実行して疎通確認します。

---
