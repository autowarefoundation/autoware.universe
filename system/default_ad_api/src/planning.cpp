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

#include "planning.hpp"

#include <motion_utils/motion_utils.hpp>

#include <string>
#include <vector>

namespace default_ad_api
{

template <class T>
void concat(std::vector<T> & v1, const std::vector<T> & v2)
{
  v1.insert(v1.end(), v2.begin(), v2.end());
}

PlanningNode::PlanningNode(const rclcpp::NodeOptions & options) : Node("planning", options)
{
  // TODO(Takagi, Isamu): remove default value
  stop_judge_distance_ = declare_parameter<double>("stop_judge_distance", 1.0);

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(pub_velocity_factors_);
  adaptor.init_pub(pub_steering_factors_);
  adaptor.init_sub(
    sub_kinematic_state_,
    [this](const localization_interface::KinematicState::Message::ConstSharedPtr msg) {
      kinematic_state_ = msg;
    });
  adaptor.init_sub(
    sub_trajectory_, [this](const planning_interface::Trajectory::Message::ConstSharedPtr msg) {
      trajectory_ = msg;
    });

  std::vector<std::string> velocity_factor_topics = {
    "/planning/velocity_factors/blind_spot",
    "/planning/velocity_factors/crosswalk",
    "/planning/velocity_factors/detection_area",
    "/planning/velocity_factors/intersection",
    "/planning/velocity_factors/merge_from_private",
    "/planning/velocity_factors/no_stopping_area",
    "/planning/velocity_factors/obstacle_stop",
    "/planning/velocity_factors/obstacle_cruise",
    "/planning/velocity_factors/occlusion_spot",
    "/planning/velocity_factors/stop_line",
    "/planning/velocity_factors/surround_obstacle",
    "/planning/velocity_factors/traffic_light",
    "/planning/velocity_factors/virtual_traffic_light",
    "/planning/velocity_factors/walkway"};

  std::vector<std::string> steering_factor_topics = {
    "/planning/steering_factor/avoidance", "/planning/steering_factor/intersection",
    "/planning/steering_factor/lane_change", "/planning/steering_factor/pull_out",
    "/planning/steering_factor/pull_over"};

  const auto on_velocity_factors = [this](const int index) {
    return [this, index](const VelocityFactorArray::ConstSharedPtr msg) {
      velocity_factors_[index] = msg;
    };
  };

  const auto on_steering_factors = [this](const int index) {
    return [this, index](const SteeringFactorArray::ConstSharedPtr msg) {
      steering_factors_[index] = msg;
    };
  };

  for (size_t index = 0; index < velocity_factor_topics.size(); ++index) {
    sub_velocity_factors_.push_back(create_subscription<VelocityFactorArray>(
      velocity_factor_topics[index], rclcpp::QoS(1), on_velocity_factors(index)));
  }
  velocity_factors_.resize(velocity_factor_topics.size());

  for (size_t index = 0; index < steering_factor_topics.size(); ++index) {
    sub_steering_factors_.push_back(create_subscription<SteeringFactorArray>(
      steering_factor_topics[index], rclcpp::QoS(1), on_steering_factors(index)));
  }
  steering_factors_.resize(steering_factor_topics.size());

  const auto rate = rclcpp::Rate(5);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() {
    using autoware_adapi_v1_msgs::msg::VelocityFactor;
    VelocityFactorArray velocity;
    SteeringFactorArray steering;
    velocity.header.stamp = now();
    steering.header.stamp = now();
    velocity.header.frame_id = "map";
    steering.header.frame_id = "map";
    for (const auto & factor : velocity_factors_) {
      if (factor) {
        concat(velocity.factors, factor->factors);
      }
    }
    for (const auto & factor : steering_factors_) {
      if (factor) {
        concat(steering.factors, factor->factors);
      }
    }
    for (auto & factor : velocity.factors) {
      if (kinematic_state_ && trajectory_ && std::isnan(factor.distance)) {
        const auto & curr_point = kinematic_state_->pose.pose.position;
        const auto & stop_point = factor.pose.position;
        const auto & points = trajectory_->points;
        factor.distance = motion_utils::calcSignedArcLength(points, curr_point, stop_point);
      }
      if ((factor.status == VelocityFactor::UNKNOWN) && (!std::isnan(factor.distance))) {
        if (factor.distance < stop_judge_distance_) {
          factor.status = VelocityFactor::STOPPED;
        } else {
          factor.status = VelocityFactor::APPROACHING;
        }
      }
    }
    pub_velocity_factors_->publish(velocity);
    pub_steering_factors_->publish(steering);
  });
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::PlanningNode)
