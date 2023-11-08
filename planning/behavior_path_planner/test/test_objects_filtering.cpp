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

#include <gtest/gtest.h>

#include "behavior_path_planner/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner/utils/utils.hpp"
#include "object_recognition_utils/predicted_path_utils.hpp"


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
    double velocity_threshold = 1.0; // eg 0.5
    bool remove_threshold = true; 

    PredictedObjects ans;
    ans = filterObjectsByVelocity(predicted_objects, velocity_threshold, remove_threshold);
    EXPECT_EQ(ans, expected_ans[1]);

    // another case
    remove_threshold = false; 

    ans = filterObjectsByVelocity(predicted_objects, velocity_threshold, remove_threshold);
    EXPECT_EQ(ans, expected_ans[0]);



}
TEST(BehaviorPathPlanningObjectsFiltering, filterObjectsByPosition)
{
    using autoware_auto_perception_msgs::msg::PredictedObject;
    using autoware_auto_perception_msgs::msg::PredictedObjects;
    using behavior_path_planner::utils::path_safety_checker::filterObjectsByPosition;
    using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
    using autoware_auto_planning_msgs::msg::PathPoint;
    using geometry_msgs::msg::Pose;

    PredictedObjects predicted_objects;
    PredictedObject predicted_object;
    PredictedObjects ans_objects;
    PredictedObjects ans_object;

    const std::vector<PathPointWithLaneId> path_points;
    //tier4_autoware_utils::createPoint(0, 0, 0);
    
    const geometry_msgs::msg::Point current_pose = tier4_autoware_utils::createPoint(1, 0, 0);
    const double forward_distance = 1.0;
    const double backward_distance = 1.0;

    
    filterObjectsByPosition(predicted_objects, path_points, current_pose, forward_distance, backward_distance);
    EXPECT_EQ(predicted_objects, ans_objects);
}