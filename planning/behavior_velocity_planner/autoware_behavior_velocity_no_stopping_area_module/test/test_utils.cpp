// Copyright 2024 TIER IV, Inc.
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

#include "../src/utils.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>

#include <gtest/gtest.h>

TEST(NoStoppingAreaTest, isTargetStuckVehicleType)
{
  using autoware::behavior_velocity_planner::no_stopping_area::is_vehicle_type;
  autoware_perception_msgs::msg::PredictedObject object;
  EXPECT_NO_FATAL_FAILURE(is_vehicle_type(object));
  autoware_perception_msgs::msg::ObjectClassification classification;
  for (const auto label : {
         autoware_perception_msgs::msg::ObjectClassification::CAR,
         autoware_perception_msgs::msg::ObjectClassification::BUS,
         autoware_perception_msgs::msg::ObjectClassification::TRUCK,
         autoware_perception_msgs::msg::ObjectClassification::TRAILER,
         autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE,
       }) {
    classification.label = label;
    object.classification = {classification};
    EXPECT_TRUE(is_vehicle_type(object));
  }
}
