// Copyright 2020 Tier IV, Inc.
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

#include "rclcpp/rclcpp.hpp"

#include "awapi_awiv_adapter/awapi_autoware_util.hpp"
#include "autoware_api_msgs/msg/lane_change_status.hpp"

namespace autoware_api
{
class AutowareIvLaneChangeStatePublisher
{
public:
  AutowareIvLaneChangeStatePublisher(rclcpp::Node& node);
  void statePublisher(const AutowareInfo & aw_info);

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  // publisher
  rclcpp::Publisher<autoware_api_msgs::msg::LaneChangeStatus>::SharedPtr pub_state_;

  void getLaneChangeAvailableInfo(
    const std_msgs::msg::Bool::ConstSharedPtr & available_ptr,
    autoware_api_msgs::msg::LaneChangeStatus * status);
  void getLaneChangeReadyInfo(
    const std_msgs::msg::Bool::ConstSharedPtr & ready_ptr,
    autoware_api_msgs::msg::LaneChangeStatus * status);
  void getCandidatePathInfo(
    const autoware_planning_msgs::msg::Path::ConstSharedPtr & path_ptr,
    autoware_api_msgs::msg::LaneChangeStatus * status);
};

}  // namespace autoware_api
