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

#include "initial_pose_adaptor.hpp"

#include "copy_vector_to_array.hpp"

#include <memory>

InitialPoseAdaptor::InitialPoseAdaptor() : Node("initial_pose_rviz_helper"), map_fit_(this)
{
  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_cli(cli_initialize_);

  sub_initial_pose_ = create_subscription<PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(1),
    std::bind(&InitialPoseAdaptor::OnInitialPose, this, std::placeholders::_1));

  rviz_particle_covariance_ = GetCovarianceParameter(this, "initialpose_particle_covariance");
}

void InitialPoseAdaptor::OnInitialPose(PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  try {
    const auto req = std::make_shared<Initialize::Service::Request>();
    req->pose.push_back(map_fit_.FitHeight(*msg));
    req->pose.back().pose.covariance = rviz_particle_covariance_;
    cli_initialize_->async_send_request(req);
  } catch (const component_interface_utils::ServiceException & error) {
    RCLCPP_ERROR_STREAM(get_logger(), error.what());
  }
}
