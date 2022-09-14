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

#include <string>
#include <vector>

namespace default_ad_api
{

bool starts_with(const std::string & s, const std::string & t)
{
  return t.size() <= s.size() && std::equal(t.begin(), t.end(), s.begin());
}

template <class T>
void concat(std::vector<T> & v1, const std::vector<T> & v2)
{
  v1.insert(v1.end(), v2.begin(), v2.end());
}

PlanningNode::PlanningNode(const rclcpp::NodeOptions & options) : Node("planning", options)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(pub_velocity_factors_);
  adaptor.init_pub(pub_steering_factors_);

  {
    const auto on_velocity_factors = [this](const int index) {
      return [this, index](const VelocityFactorArray::ConstSharedPtr msg) {
        velocity_factors_[index] = msg;
      };
    };

    int index = 0;
    for (const auto & topic : get_topic_names_and_types()) {
      if (starts_with(topic.first, "/planning/velocity_factors")) {
        RCLCPP_INFO_STREAM(get_logger(), index << " " << topic.first);
        sub_velocity_factors_.push_back(create_subscription<VelocityFactorArray>(
          topic.first, rclcpp::QoS(1), on_velocity_factors(index++)));
      }
    }
    velocity_factors_.resize(index);
  }

  const auto rate = rclcpp::Rate(5);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() {
    VelocityFactorArray message;
    message.header.stamp = now();
    message.header.frame_id = "map";

    int index = 0;
    for (const auto & factor : velocity_factors_) {
      if (factor) {
        concat(message.factors, factor->factors);
        RCLCPP_INFO_STREAM(get_logger(), index++ << " " << factor->factors.size());
      }
    }
    pub_velocity_factors_->publish(message);
  });
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::PlanningNode)
