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

#include "dummy_pointcloud_publisher/dummy_pointcloud_publisher_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace dummy_pointcloud_publisher
{

DummyPointCloudPublisher::DummyPointCloudPublisher(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("dummy_pointcloud_publisher", options)
{
  // --- parameters ---
  topic_names_ =
    this->declare_parameter<std::vector<std::string>>("topic_names");
  frame_ids_ = this->declare_parameter<std::vector<std::string>>("frame_ids");
  rate_hz_ = this->declare_parameter<double>("rate_hz");

  if (topic_names_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'topic_names' is empty.");
  }

  const size_t n = topic_names_.size();
  const auto frames = expandOrValidate<std::string>(frame_ids_, n, "frame_ids");

  // --- QoS ---
  const auto qos = rclcpp::SensorDataQoS();

  // --- create publishers & templates ---
  streams_.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    Stream s;
    s.topic_name = topic_names_[i];
    s.frame_id = frames[i];

    s.pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      s.topic_name,
      qos);
    s.msg = makePointCloudTemplate(s.frame_id);

    streams_.push_back(std::move(s));

    RCLCPP_INFO(
      this->get_logger(), "Stream[%zu]: topic=%s frame_id=%s", i,
      streams_[i].topic_name.c_str(), streams_[i].frame_id.c_str());
  }

  // --- single timer publishes all topics ---
  const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&DummyPointCloudPublisher::onTimer, this));

  RCLCPP_INFO(
    this->get_logger(), "Started. rate_hz=%.3f streams=%zu", rate_hz_,
    streams_.size());
}

void DummyPointCloudPublisher::onTimer()
{
  const auto stamp = this->now();
  for (auto & s : streams_) {
    // update only the stamp
    s.msg.header.stamp = stamp;
    s.pub->publish(s.msg);
  }
}

sensor_msgs::msg::PointCloud2 DummyPointCloudPublisher::makePointCloudTemplate(
  const std::string & frame_id) const
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_id;

  // minimum required fields for PointCloud2: x,y,z float32
  msg.height = 1;
  msg.width = 0;

  msg.is_bigendian = false;
  msg.is_dense = true;

  msg.fields.resize(6);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;

  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
  msg.fields[3].count = 1;

  msg.fields[4].name = "return_type";
  msg.fields[4].offset = 13;
  msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  msg.fields[4].count = 1;

  msg.fields[5].name = "channel";
  msg.fields[5].offset = 14;
  msg.fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
  msg.fields[5].count = 1;

  msg.point_step = 16;
  msg.row_step = 0;

  msg.data.resize(msg.row_step);

  return msg;
}

template<class T>
std::vector<T> DummyPointCloudPublisher::expandOrValidate(
  const std::vector<T> & v, size_t n, const std::string & param_name) const
{
  if (v.size() == 1) {
    return std::vector<T>(n, v[0]);
  }
  if (v.size() == n) {
    return v;
  }
  throw std::runtime_error(
          "parameter '" + param_name +
          "' size must be 1, or topic_names.size()");
}

}  // namespace dummy_pointcloud_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(
  dummy_pointcloud_publisher::DummyPointCloudPublisher)
