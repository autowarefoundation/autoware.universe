// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner/utils/utils.hpp"
#include "object_recognition_utils/predicted_path_utils.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>

using namespace behavior_path_planner::utils::path_safety_checker;

TEST(BehaviorPathPlanningObjectsFiltering, filterObjectsByVelocity)
{
  using autoware_auto_perception_msgs::msg::PredictedObject;
  using autoware_auto_perception_msgs::msg::PredictedObjects;
  using behavior_path_planner::utils::path_safety_checker::filterObjectsByVelocity;

  PredictedObjects predicted_objects;
  PredictedObject predicted_object;
  PredictedObjects expected_ans[2];

  predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x = 1.0;
  predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y = 1.0;

  expected_ans[0].objects.push_back(predicted_object);
  predicted_objects.objects.push_back(predicted_object);

  predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x = 0.5;
  predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y = 0.5;

  expected_ans[1].objects.push_back(predicted_object);
  predicted_objects.objects.push_back(predicted_object);

  // case function has boolian flag "remove_above_threshold" = true
  double velocity_threshold = 1.0;  // eg 0.5
  bool remove_threshold = true;

  PredictedObjects ans;
  ans = filterObjectsByVelocity(predicted_objects, velocity_threshold, remove_threshold);
  EXPECT_EQ(ans, expected_ans[1]);

  // another case
  remove_threshold = false;

  ans = filterObjectsByVelocity(predicted_objects, velocity_threshold, remove_threshold);
  EXPECT_EQ(ans, expected_ans[0]);
}

TEST(BehaviorPathPlanningObjectsFiltering, filterObjectsByClass)
{
    using autoware_auto_perception_msgs::msg::PredictedObject;
    using autoware_auto_perception_msgs::msg::PredictedObjects;
    using autoware_auto_perception_msgs::msg::ObjectClassification;
    using behavior_path_planner::utils::path_safety_checker::filterObjectsByClass;
    

    PredictedObjects predicted_objects;
    PredictedObject predicted_object;
    PredictedObjects expected_ans[2];

    ObjectClassification car_class;
    car_class.label = ObjectClassification::CAR;
    car_class.probability = 1.0;

    predicted_object.classification.push_back(car_class);

    expected_ans[0].objects.push_back(predicted_object);
    predicted_objects.objects.push_back(predicted_object);

    car_class.label = ObjectClassification::UNKNOWN;
    car_class.probability = 0.0;
    predicted_object.classification.push_back(car_class);

    expected_ans[1].objects.push_back(predicted_object);
    predicted_objects.objects.push_back(predicted_object);

    ObjectTypesToCheck target_types;
    target_types.check_car = true;
    target_types.check_truck = false;
    target_types.check_bus = false;
    target_types.check_trailer = false;
    target_types.check_bicycle = false;
    target_types.check_unknown = false;
    target_types.check_motorcycle = false;
    target_types.check_pedestrian = false;
    

    filterObjectsByClass(predicted_objects, target_types);

    // size_t size_of_predicted_objects = predicted_objects.objects.size();
    // size_t size_of_expected_ans = expected_ans[0].objects.size();
    // std::cout << "Size of predicted_objects: " << size_of_predicted_objects << std::endl;
    // std::cout << "Size of expected0_objects: " << size_of_expected_ans << std::endl;
    
    EXPECT_EQ(predicted_objects, expected_ans[0]);
}