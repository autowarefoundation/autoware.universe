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

namespace pose_estimator_manager::sub_manager
{
class SubManagerArTag : public BasePoseEstimatorSubManager
{
public:
  using Image = sensor_msgs::msg::Image;
  using SetBool = std_srvs::srv::SetBool;

  explicit SubManagerArTag(rclcpp::Node * node) : BasePoseEstimatorSubManager(node)
  {
    ar_tag_is_enabled_ = true;

    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    using std::placeholders::_1;
    auto on_image = std::bind(&SubManagerArTag::on_image, this, _1);
    sub_image_ = node->create_subscription<Image>("~/input/artag/image", qos, on_image);
    pub_image_ = node->create_publisher<Image>("~/output/artag/image", qos);
  }

  void set_enable(bool enabled) override { ar_tag_is_enabled_ = enabled; }

protected:
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

private:
  bool ar_tag_is_enabled_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;

  void on_image(Image::ConstSharedPtr msg)
  {
    if (ar_tag_is_enabled_) {
      pub_image_->publish(*msg);
    }
  }
};
}  // namespace pose_estimator_manager::sub_manager

#endif  // POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_ARTAG_HPP_
