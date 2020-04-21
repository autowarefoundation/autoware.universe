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
#include <cstdlib>
#include "microstrain_mips/SetMagDipAdaptiveVals.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "set_mag_dip_adaptive_vals_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_mips::SetMagDipAdaptiveVals>("SetMagDipAdaptiveVals");
  microstrain_mips::SetMagDipAdaptiveVals srv;

  srv.request.enable = atoll(argv[1]);
  srv.request.low_pass_cutoff = atoll(argv[2]);
  srv.request.min_1sigma = atoll(argv[3]);
  srv.request.high_limit = atoll(argv[4]);
  srv.request.high_limit_1sigma = atoll(argv[5]);

  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("success");
    }
  } else {
    ROS_INFO("Failed to call service");
  }
  return 0;
}
