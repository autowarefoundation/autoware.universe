// Copyright 2020 Autoware Foundation
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

#include "pose_initializer_core.hpp"

#include "copy_vector_to_array.hpp"
#include "gnss_module.hpp"
#include "ndt_module.hpp"
#include "stop_check_module.hpp"

#include <memory>
#include <vector>

PoseInitializer::PoseInitializer() : Node("pose_initializer")
{
  const auto node = component_interface_utils::NodeAdaptor(this);
  group_srv_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  node.init_pub(pub_state_);
  node.init_srv(srv_initialize_, BIND_SERVICE(this, OnInitialize), group_srv_);
  pub_reset_ = create_publisher<PoseWithCovarianceStamped>("pose_reset", 1);

  output_pose_covariance_ = GetCovarianceParameter(this, "output_pose_covariance");
  gnss_particle_covariance_ = GetCovarianceParameter(this, "gnss_particle_covariance");

  if (declare_parameter<bool>("gnss_enabled")) {
    gnss_ = std::make_unique<GnssModule>(this);
  }
  if (declare_parameter<bool>("ndt_enabled")) {
    ndt_ = std::make_unique<NdtModule>(this);
  }
  if (declare_parameter<bool>("stop_check_enabled")) {
    // Add 1.0 sec margin for twist buffer.
    stop_check_duration_ = declare_parameter<double>("stop_check_duration");
    stop_check_ = std::make_unique<StopCheckModule>(this, stop_check_duration_ + 1.0);
  }
  ChangeState(State::Message::UNINITIALIZED);
}

PoseInitializer::~PoseInitializer()
{
  // to delete gnss module
}

void PoseInitializer::ChangeState(State::Message::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}

void PoseInitializer::OnInitialize(API_SERVICE_ARG(Initialize, req, res))
{
  // NOTE: This function is not executed during initialization because mutually exclusive.
  if (stop_check_ && !stop_check_->isVehicleStopped(stop_check_duration_)) {
    throw ServiceException(
      Initialize::Service::Response::ERROR_UNSAFE, "The vehicle is not stopped.");
  }
  try {
    ChangeState(State::Message::INITIALIZING);
    auto pose = req->pose.empty() ? GetGnssPose() : req->pose.front();
    if (ndt_) {
      pose = ndt_->AlignPose(pose);
    }
    pose.pose.covariance = output_pose_covariance_;
    pub_reset_->publish(pose);
    res->status.success = true;
    ChangeState(State::Message::INITIALIZED);
  } catch (const ServiceException & error) {
    res->status = error.status();
    ChangeState(State::Message::UNINITIALIZED);
  }
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseInitializer::GetGnssPose()
{
  if (gnss_) {
    PoseWithCovarianceStamped pose = gnss_->GetPose();
    pose.pose.covariance = gnss_particle_covariance_;
    return pose;
  }
  throw ServiceException(
    Initialize::Service::Response::ERROR_GNSS_SUPPORT, "GNSS is not supported.");
}
