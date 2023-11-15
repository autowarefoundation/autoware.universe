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

    std::vector<PathPointWithLaneId> sample_path_points;

    float initial_pose_value = 0.0; 
    float pose_increment = 1.0;
    size_t point_sample = 10;
    for (size_t idx = 0; idx < point_sample; ++idx) {
        PathPoint point;
        point.pose.position.x = std::exchange(initial_pose_value, initial_pose_value + pose_increment);
        point.pose.position.y = 0.0;
        point.pose.position.z = 0.0;
        point.longitudinal_velocity_mps = 0.1;  // [m/s]
        point.heading_rate_rps = 0.0;           // [rad/s]
        point.is_final = (idx == point_sample - 1);

        PathPointWithLaneId path_point_with_lane_id;
        path_point_with_lane_id.point = point;
        path_point_with_lane_id.lane_ids = std::vector<int64_t>();

        sample_path_points.push_back(path_point_with_lane_id);
    }

    PredictedObjects predicted_objects;
    PredictedObject predicted_object[2];
    PredictedObjects ans_objects;

    geometry_msgs::msg::Point current_pose = tier4_autoware_utils::createPoint(0, 0, 0);
    double forward_distance = 1.0;
    double backward_distance = 1.0;

    //no objects case
    filterObjectsByPosition(predicted_objects, sample_path_points, current_pose, forward_distance, backward_distance);
    EXPECT_EQ(predicted_objects, ans_objects);


    current_pose = tier4_autoware_utils::createPoint(0, 0, 0);
    forward_distance = 5.0;
    backward_distance = 5.0;
    
    //objects passed case
    predicted_object[0].kinematics.initial_pose_with_covariance.pose.position.x = 2.0;
    predicted_object[0].kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
    predicted_objects.objects.push_back(predicted_object[0]);
    ans_objects.objects.push_back(predicted_object[0]);
    
    filterObjectsByPosition(predicted_objects, sample_path_points, current_pose, forward_distance, backward_distance);
    EXPECT_EQ(predicted_objects, ans_objects);
    

    //objects didn't pass case
    predicted_object[0].kinematics.initial_pose_with_covariance.pose.position.x = 6.0;
    predicted_object[0].kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
    predicted_objects.objects.push_back(predicted_object[0]);
    
    filterObjectsByPosition(predicted_objects, sample_path_points, current_pose, forward_distance, backward_distance);
    EXPECT_EQ(predicted_objects, ans_objects);
    
    //std::cout << "number of objects: " << predicted_objects.objects.size() << std::endl;


    
}