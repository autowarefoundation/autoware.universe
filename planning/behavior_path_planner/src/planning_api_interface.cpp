// Copyright 2022 Tier IV, Inc.
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

#include "behavior_path_planner/planning_api_interface.hpp"

namespace planning_api_interface
{
PlanningAPIInterface::PlanningAPIInterface(rclcpp::Node * node, const std::string & name)
: logger_{node->get_logger().get_child("PlanningAPI[" + name + "]")}
{
  // Publisher
  pub_steering_factors_ =
    node->create_publisher<SteeringFactorArray>("/planning/api/" + name + "/steering_factor", 1);
}

void PlanningAPIInterface::publishSteeringFactor(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  registered_steering_factors_.header.stamp = stamp;
  pub_steering_factors_->publish(registered_steering_factors_);
}

void PlanningAPIInterface::updateSteeringFactor(
  const std::vector<Pose> & pose, const std::vector<double> distance, const uint16_t type,
  const uint16_t direction, const uint16_t status, const std::string detail)
{
  std::lock_guard<std::mutex> lock(mutex_);
  SteeringFactor factor;
  factor.pose = pose;
  std::vector<float> convert_distance(distance.begin(), distance.end());
  factor.distance = convert_distance;
  factor.type = type;
  factor.direction = direction;
  factor.status = status;
  factor.detail = detail;
  registered_steering_factors_.factors = {factor};
}

void PlanningAPIInterface::clearSteeringFactors()
{
  std::lock_guard<std::mutex> lock(mutex_);
  registered_steering_factors_.factors.clear();
}

rclcpp::Logger PlanningAPIInterface::getLogger() const { return logger_; }

}  // namespace planning_api_interface
