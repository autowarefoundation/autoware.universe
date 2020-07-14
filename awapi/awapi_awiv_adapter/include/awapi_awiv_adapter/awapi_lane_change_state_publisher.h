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

#include <awapi_awiv_adapter/LaneChangeStatus.h>
#include <awapi_awiv_adapter/awapi_autoware_util.h>

namespace autoware_api
{
class AutowareIvLaneChangeStatePublisher
{
public:
  AutowareIvLaneChangeStatePublisher();
  void statePublisher(const AutowareInfo & aw_info);

private:
  // node handle
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // publisher
  ros::Publisher pub_state_;

  void getLaneChangeAvailableInfo(
    const std_msgs::Bool::ConstPtr & available_ptr, awapi_awiv_adapter::LaneChangeStatus * status);
  void getLaneChangeReadyInfo(
    const std_msgs::Bool::ConstPtr & ready_ptr, awapi_awiv_adapter::LaneChangeStatus * status);
  void getCandidatePathInfo(
    const autoware_planning_msgs::Path::ConstPtr & path_ptr,
    awapi_awiv_adapter::LaneChangeStatus * status);
};

}  // namespace autoware_api