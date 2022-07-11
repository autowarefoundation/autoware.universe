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

#ifndef MRM_COMFORTABLE_STOP_OPERATOR__MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_
#define MRM_COMFORTABLE_STOP_OPERATOR__MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_

// Core
#include <memory>

// Autoware
#include <autoware_ad_api_msgs/srv/mrm_operation.hpp>
#include <autoware_ad_api_msgs/msg/mrm_status.hpp>

// ROS2 core
#include <rclcpp/rclcpp.hpp>


class MRMComfortableStopOperator : public rclcpp::Node
{
public:
  MRMComfortableStopOperator();

private:

};

#endif  // MRM_COMFORTABLE_STOP_OPERATOR__MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_
