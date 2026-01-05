// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DUMMY_POINTCLOUD_PUBLISHER__DUMMY_POINTCLOUD_PUBLISHER_NODE_HPP_
#define DUMMY_POINTCLOUD_PUBLISHER__DUMMY_POINTCLOUD_PUBLISHER_NODE_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace dummy_pointcloud_publisher
{

class DummyPointCloudPublisher : public rclcpp::Node
{
public:
  explicit DummyPointCloudPublisher(const rclcpp::NodeOptions & options);

private:
  struct Stream
  {
    std::string topic_name;
    std::string frame_id;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    sensor_msgs::msg::PointCloud2 msg;
  };

  void onTimer();
  sensor_msgs::msg::PointCloud2
  makePointCloudTemplate(const std::string & frame_id) const;

  template<class T>
  std::vector<T> expandOrValidate(
    const std::vector<T> & v, size_t n,
    const std::string & param_name) const;

  // parameters
  std::vector<std::string> topic_names_;
  std::vector<std::string> frame_ids_;

  double rate_hz_;

  // streams
  std::vector<Stream> streams_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dummy_pointcloud_publisher

#endif  // DUMMY_POINTCLOUD_PUBLISHER__DUMMY_POINTCLOUD_PUBLISHER_NODE_HPP_
