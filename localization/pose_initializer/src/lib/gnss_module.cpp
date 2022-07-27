// Copyright 2022 Autoware Foundation
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

#include "gnss_module.hpp"

#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp/exceptions.hpp>

GnssModule::GnssModule(rclcpp::Node * node) : map_fit_(node)
{
  sub_gnss_pose_ = node->create_subscription<PoseWithCovarianceStamped>(
    "gnss_pose_cov", 1, std::bind(&GnssModule::OnGnssPose, this, std::placeholders::_1));

  clock_ = node->get_clock();
  timeout_ = node->declare_parameter<double>("gnss_pose_timeout");
}

void GnssModule::OnGnssPose(PoseWithCovarianceStamped::ConstSharedPtr msg) { gnss_pose_ = msg; }

PoseWithCovarianceStamped GnssModule::GetPose() const
{
  using Initialize = localization_interface::Initialize;

  if (!gnss_pose_) {
    throw component_interface_utils::ServiceException(
      Initialize::Service::Response::ERROR_GNSS, "The GNSS pose has not arrived.");
  }

  const auto elapsed = rclcpp::Time(gnss_pose_->header.stamp) - clock_->now();
  if (timeout_ < elapsed.seconds()) {
    throw component_interface_utils::ServiceException(
      Initialize::Service::Response::ERROR_GNSS, "The GNSS pose is out of date.");
  }
  return *gnss_pose_;
}
