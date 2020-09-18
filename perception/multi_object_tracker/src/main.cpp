/*
 * Copyright 2018 Yukihiro Saito. All rights reserved.
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
 *
 *
 * v1.0 Yukihiro Saito
 */

#include <ros/ros.h>
#include "multi_object_tracker/node.hpp"

// #include "multi_object_tracker/data_association/data_association.hpp"

int main(int argc, char ** argv)
{
  // Eigen::MatrixXd score(4, 3);
  // std::unordered_map<int, int> direct_assignment;
  // std::unordered_map<int, int> reverse_assignment;
  // score << 0, 10, 0,
  //     4, 5, 6,
  //     10, 1, 0,
  //     0, 100, 0;
  // DataAssociation::assign(score, direct_assignment, reverse_assignment);
#if 1
  ros::init(argc, argv, "multi_object_tracker");
  MultiObjectTrackerNode node;
  ros::spin();
#endif

  return 0;
}
