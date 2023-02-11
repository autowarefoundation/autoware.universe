// Copyright 2023 TIER IV, Inc. All rights reserved.
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

#include <memory>

#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <path_footprint/display.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2/utils.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowarePathWithLaneIdFootprintDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowarePathFootprintDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareTrajectoryFootprintDisplay, rviz_common::Display)
