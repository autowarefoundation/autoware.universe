// Copyright 2022 TIER IV, Inc.
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

#include <memory>
#include <string>
#include <vector>

namespace default_ad_api_helpers
{
template <class ServiceT>
using Future = typename rclcpp::Client<ServiceT>::SharedFuture;

std::array<double, 36> GetCovarianceParameter(rclcpp::Node * node, const std::string & name)
{
  const auto vector = node->template declare_parameter<std::vector<double>>(name);
  if (vector.size() != 36) {
    throw std::invalid_argument("The covariance parameter size is not 36.");
  }
  std::array<double, 36> array;
  std::copy_n(vector.begin(), array.size(), array.begin());
  return array;
}

InitialPoseAdaptor::InitialPoseAdaptor() : Node("initial_pose_adaptor")
{
  rviz_particle_covariance_ = GetCovarianceParameter(this, "initial_pose_particle_covariance");
  cli_map_fit_ = create_client<RequestHeightFitting>("fit_map_height");
  sub_initial_pose_ = create_subscription<PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(1),
    std::bind(&InitialPoseAdaptor::OnInitialPose, this, std::placeholders::_1));

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_cli(cli_initialize_);
}

void InitialPoseAdaptor::OnInitialPose(PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  const auto req = std::make_shared<RequestHeightFitting::Request>();
  req->pose_with_covariance = *msg;
  cli_map_fit_->async_send_request(req, [this](Future<RequestHeightFitting> future) {
    const auto req = std::make_shared<Initialize::Service::Request>();
    req->pose.push_back(future.get()->pose_with_covariance);
    req->pose.back().pose.covariance = rviz_particle_covariance_;
    cli_initialize_->async_send_request(req);
  });
}

}  // namespace default_ad_api_helpers

#include "macros/create_node.hpp"
CREATE_SINGLE_THREAD_NODE(default_ad_api_helpers::InitialPoseAdaptor)
