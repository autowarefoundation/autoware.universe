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

#ifndef POSE_ESTIMATOR_MANAGER__BASE_POSE_ESTIMATOR_SUB_MANAGER_HPP_
#define POSE_ESTIMATOR_MANAGER__BASE_POSE_ESTIMATOR_SUB_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace multi_pose_estimator
{
class BasePoseEstimatorSubManager
{
public:
  using SharedPtr = std::shared_ptr<BasePoseEstimatorSubManager>;

  explicit BasePoseEstimatorSubManager(rclcpp::Node * node) : logger_(node->get_logger()) {}

  void enable() { set_enable(true); }
  void disable() { set_enable(false); }

  virtual void set_enable(bool enabled) = 0;

protected:
  rclcpp::Logger logger_;

private:
};
}  // namespace multi_pose_estimator

#endif  // POSE_ESTIMATOR_MANAGER__BASE_POSE_ESTIMATOR_SUB_MANAGER_HPP_
