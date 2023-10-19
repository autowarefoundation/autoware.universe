// Copyright 2023 Autoware Foundation
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

#ifndef POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_NDT_HPP_
#define POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_NDT_HPP_
#include "pose_estimator_manager/base_pose_estimator_sub_manager.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace multi_pose_estimator
{
class SubManagerNdt : public BasePoseEstimatorSubManager
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  SubManagerNdt(rclcpp::Node * node) : BasePoseEstimatorSubManager(node)
  {
    using std::placeholders::_1;
    auto on_pointcloud = std::bind(&SubManagerNdt::on_pointcloud, this, _1);

    sub_pointcloud_ = node->create_subscription<PointCloud2>(
      "~/input/pointcloud", rclcpp::SensorDataQoS(), on_pointcloud);
    pub_pointcloud_ = node->create_publisher<PointCloud2>(
      "~/output/pointcloud", rclcpp::SensorDataQoS().keep_last(10));

    ndt_is_enabled_ = true;
  }

  void set_enable(bool enabled) override { ndt_is_enabled_ = enabled; }

private:
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_;
  bool ndt_is_enabled_;

  void on_pointcloud(PointCloud2::ConstSharedPtr msg)
  {
    if (ndt_is_enabled_) pub_pointcloud_->publish(*msg);
  }
};
}  // namespace multi_pose_estimator

#endif  // POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_NDT_HPP_
