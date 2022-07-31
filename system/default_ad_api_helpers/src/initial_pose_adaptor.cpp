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

namespace default_ad_api_helpers
{

InitialPoseAdaptor::InitialPoseAdaptor(const rclcpp::NodeOptions & options)
: Node("initial_pose_adaptor", options)
{
  rviz_particle_covariance_ = GetCovarianceParameter(this, "initial_pose_particle_covariance");
  cli_map_fit_ = create_client<RequestHeightFitting>("fit_map_height");
  sub_initial_pose_ = create_subscription<PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(1),
    std::bind(&InitialPoseAdaptor::OnInitialPose, this, std::placeholders::_1));

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_cli(cli_initialize_);
}

void Dump(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  const auto & p = pose.pose.pose.position;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "x: " << p.x);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "y: " << p.y);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "z: " << p.z);
}

template <class ServiceT>
using Future = typename rclcpp::Client<ServiceT>::SharedFuture;

void InitialPoseAdaptor::OnInitialPose(PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "==================================");
  Dump(*msg);

  const auto req = std::make_shared<RequestHeightFitting::Request>();
  req->pose_with_covariance = *msg;
  cli_map_fit_->async_send_request(req, [this](Future<RequestHeightFitting> future) {
    Dump(future.get()->pose_with_covariance);
    const auto req = std::make_shared<Initialize::Service::Request>();
    req->pose.push_back(future.get()->pose_with_covariance);
    req->pose.back().pose.covariance = rviz_particle_covariance_;
    cli_initialize_->async_send_request(req);
  });
}

}  // namespace default_ad_api_helpers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api_helpers::InitialPoseAdaptor)
