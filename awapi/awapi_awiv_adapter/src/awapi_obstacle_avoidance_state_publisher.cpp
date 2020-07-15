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

#include <awapi_awiv_adapter/awapi_obstacle_avoidance_state_publisher.h>

namespace autoware_api
{
AutowareIvObstacleAvoidanceStatePublisher::AutowareIvObstacleAvoidanceStatePublisher()
: nh_(), pnh_("~")
{
  // publisher
  pub_state_ =
    pnh_.advertise<autoware_api_msgs::ObstacleAvoidanceStatus>("output/obstacle_avoid_status", 1);
}

void AutowareIvObstacleAvoidanceStatePublisher::statePublisher(const AutowareInfo & aw_info)
{
  autoware_api_msgs::ObstacleAvoidanceStatus status;

  //input header
  status.header.frame_id = "base_link";
  status.header.stamp = ros::Time::now();

  // get all info
  getObstacleAvoidReadyInfo(aw_info.obstacle_avoid_ready_ptr, &status);
  getCandidatePathInfo(aw_info.obstacle_avoid_candidate_ptr, &status);

  // publish info
  pub_state_.publish(status);
}

void AutowareIvObstacleAvoidanceStatePublisher::getObstacleAvoidReadyInfo(
  const std_msgs::Bool::ConstPtr & ready_ptr, autoware_api_msgs::ObstacleAvoidanceStatus * status)
{
  if (!ready_ptr) {
    ROS_WARN_STREAM_THROTTLE(
      5.0, "[AutowareIvObstacleAvoidanceStatePublisher] obstacle_avoidance_ready is nullptr");
    return;
  }

  status->obstacle_avoidance_ready = ready_ptr->data;
}

void AutowareIvObstacleAvoidanceStatePublisher::getCandidatePathInfo(
  const autoware_planning_msgs::Trajectory::ConstPtr & path_ptr,
  autoware_api_msgs::ObstacleAvoidanceStatus * status)
{
  if (!path_ptr) {
    ROS_WARN_STREAM_THROTTLE(
      5.0,
      "[AutowareIvObstacleAvoidanceStatePublisher] obstacle_avoidance_candidate_path is "
      "nullptr");
    return;
  }

  status->candidate_path = *path_ptr;
}

}  // namespace autoware_api
