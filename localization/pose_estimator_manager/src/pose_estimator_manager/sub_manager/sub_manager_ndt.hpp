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

#include <memory>

namespace pose_estimator_manager::sub_manager
{
class SubManagerNdt : public BasePoseEstimatorSubManager
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  explicit SubManagerNdt(rclcpp::Node * node, const std::shared_ptr<const SharedData> shared_data)
  : BasePoseEstimatorSubManager(node, shared_data)
  {
    ndt_is_enabled_ = true;
    pub_pointcloud_ = node->create_publisher<PointCloud2>(
      "~/output/ndt/pointcloud", rclcpp::SensorDataQoS().keep_last(10));

    shared_data_->ndt_input_points.set_callback([this](PointCloud2::ConstSharedPtr msg) -> void {
      if (ndt_is_enabled_) {
        pub_pointcloud_->publish(*msg);
      }
    });
  }

  void set_enable(bool enabled) override { ndt_is_enabled_ = enabled; }

  // void callback() override
  // {
  //   if (!shared_data_->ndt_input_points.updated) {
  //     return;
  //   }
  //   if (ndt_is_enabled_) {
  //     pub_pointcloud_->publish(*shared_data_->ndt_input_points());
  //   }
  // }

private:
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_;
  bool ndt_is_enabled_;
};
}  // namespace pose_estimator_manager::sub_manager

#endif  // POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_NDT_HPP_
