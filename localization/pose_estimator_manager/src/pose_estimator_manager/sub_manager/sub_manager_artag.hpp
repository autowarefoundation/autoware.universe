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

#ifndef POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_ARTAG_HPP_
#define POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_ARTAG_HPP_

#include "pose_estimator_manager/base_pose_estimator_sub_manager.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <memory>
namespace pose_estimator_manager::sub_manager
{
class SubManagerArTag : public BasePoseEstimatorSubManager
{
public:
  using Image = sensor_msgs::msg::Image;
  using SetBool = std_srvs::srv::SetBool;

  explicit SubManagerArTag(rclcpp::Node * node, const std::shared_ptr<const SharedData> shared_data)
  : BasePoseEstimatorSubManager(node, shared_data)
  {
    ar_tag_is_enabled_ = true;
    pub_image_ = node->create_publisher<Image>("~/output/artag/image", rclcpp::SensorDataQoS());

    shared_data_->artag_input_image.set_callback([this](Image::ConstSharedPtr msg) -> void {
      if (ar_tag_is_enabled_) {
        pub_image_->publish(*msg);
      }
    });
  }

  void set_enable(bool enabled) override { ar_tag_is_enabled_ = enabled; }

  // void callback() override
  // {
  //   if (!shared_data_->artag_input_image.updated) {
  //     return;
  //   }
  //   if (ar_tag_is_enabled_) {
  //     pub_image_->publish(*shared_data_->artag_input_image());
  //   }
  // }

protected:
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

private:
  bool ar_tag_is_enabled_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
};
}  // namespace pose_estimator_manager::sub_manager

#endif  // POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_ARTAG_HPP_
