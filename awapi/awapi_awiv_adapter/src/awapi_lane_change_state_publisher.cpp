/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <awapi_awiv_adapter/awapi_lane_change_state_publisher.h>

namespace autoware_api
{
AutowareIvLaneChangeStatePublisher::AutowareIvLaneChangeStatePublisher() : nh_(), pnh_("~")
{
  // publisher
  pub_state_ = pnh_.advertise<autoware_api_msgs::LaneChangeStatus>("output/lane_change_status", 1);
}

void AutowareIvLaneChangeStatePublisher::statePublisher(const AutowareInfo & aw_info)
{
  autoware_api_msgs::LaneChangeStatus status;

  //input header
  status.header.frame_id = "base_link";
  status.header.stamp = ros::Time::now();

  // get all info
  getLaneChangeAvailableInfo(aw_info.lane_change_available_ptr, &status);
  getLaneChangeReadyInfo(aw_info.lane_change_ready_ptr, &status);
  getCandidatePathInfo(aw_info.lane_change_candidate_ptr, &status);

  // publish info
  pub_state_.publish(status);
}

void AutowareIvLaneChangeStatePublisher::getLaneChangeAvailableInfo(
  const std_msgs::Bool::ConstPtr & available_ptr, autoware_api_msgs::LaneChangeStatus * status)
{
  if (!available_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(
      5.0, "[AutowareIvLaneChangeStatePublisher] lane change available is nullptr");
    return;
  }

  // get lane change available info
  status->force_lane_change_available = available_ptr->data;
}

void AutowareIvLaneChangeStatePublisher::getLaneChangeReadyInfo(
  const std_msgs::Bool::ConstPtr & ready_ptr, autoware_api_msgs::LaneChangeStatus * status)
{
  if (!ready_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(
      5.0, "[AutowareIvLaneChangeStatePublisher] lane change ready is nullptr");
    return;
  }

  // get lane change available info
  status->lane_change_ready = ready_ptr->data;
}

void AutowareIvLaneChangeStatePublisher::getCandidatePathInfo(
  const autoware_planning_msgs::Path::ConstPtr & path_ptr,
  autoware_api_msgs::LaneChangeStatus * status)
{
  if (!path_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(
      5.0,
      "[AutowareIvLaneChangeStatePublisher] lane_change_candidate_path is "
      "nullptr");
    return;
  }

  status->candidate_path = *path_ptr;
}

}  // namespace autoware_api
