// Copyright 2023 TIER IV, Inc.
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

#pragma once
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <optional>

// In the future, YabLoc will switch with NDT on the fly.
// This class was developed to make it, but it does not work now.
// At that time, YabLoc is  switched depending on whether the image topic was flowing or not.

namespace pcdless::modularized_particle_filter
{
struct SwapModeAdaptor
{
  using Image = sensor_msgs::msg::Image;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  SwapModeAdaptor(rclcpp::Node * node) : logger_(rclcpp::get_logger("swap_adaptor"))
  {
    auto on_ekf_pose = [this](const PoseCovStamped & pose) -> void { init_pose_opt_ = pose; };
    auto on_image = [this](const Image & msg) -> void {
      stamp_opt_ = rclcpp::Time(msg.header.stamp);
    };

    sub_image_ =
      node->create_subscription<Image>("/sensing/camera/undistorted/image_raw", 1, on_image);
    sub_pose_ = node->create_subscription<PoseCovStamped>(
      "/localizatrion/pose_with_covariance", 1, on_ekf_pose);
    clock_ = node->get_clock();

    state_is_active = false;
    state_is_activating = false;
  }

  std::optional<rclcpp::Time> stamp_opt_{std::nullopt};
  std::optional<PoseCovStamped> init_pose_opt_{std::nullopt};
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  bool state_is_active;
  bool state_is_activating;

  PoseCovStamped init_pose() { return init_pose_opt_.value(); }

  bool should_call_initialize()
  {
    if (state_is_activating && init_pose_opt_.has_value()) {
      if (!state_is_active) {
        state_is_active = true;
        state_is_activating = false;
        return true;
      }
    }
    return false;
  }

  bool should_keep_update()
  {
    if (!stamp_opt_.has_value()) {
      RCLCPP_INFO_STREAM_THROTTLE(logger_, *clock_, 1000, "yabloc should stop");
      state_is_active = false;
      return false;
    }

    const double dt = (clock_->now() - stamp_opt_.value()).seconds();
    if (dt > 3) {
      RCLCPP_INFO_STREAM_THROTTLE(logger_, *clock_, 1000, "yabloc should stop");
      state_is_active = false;
      return false;
    } else {
      RCLCPP_INFO_STREAM_THROTTLE(logger_, *clock_, 1000, "yabloc should keep estimation");
      state_is_activating = true;
      return true;
    }
  }
};

}  // namespace pcdless::modularized_particle_filter