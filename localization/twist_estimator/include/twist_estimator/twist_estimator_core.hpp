// Copyright 2015-2019 Autoware Foundation
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

#ifndef TWIST_ESTIMATOR__TWIST_ESTIMATOR_CORE_HPP_
#define TWIST_ESTIMATOR__TWIST_ESTIMATOR_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

class TwistEstimator : public rclcpp::Node
{
public:
  TwistEstimator();
  ~TwistEstimator();

private:
  void callbackTwistWithCovariance(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_with_cov_msg_ptr);
};

#endif  // TWIST_ESTIMATOR__TWIST_ESTIMATOR_CORE_HPP_
