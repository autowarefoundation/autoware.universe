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


#include "behavior_path_planner/utils/utils.hpp"
#include "object_recognition_utils/predicted_path_utils.hpp"
#include "behavior_path_planner/utils/path_safety_checker/objects_filtering.hpp"


using namespace behavior_path_planner::utils::path_safety_checker;


TEST(BehaviorPathPlanningObjectsFiltering, filterObjectsByVelocity)
{
    using behavior_path_planner::utils::path_safety_checker::filterObjectsByVelocity;
    
    
    PredictedObjects predicted_objects; 
    predicted_objects = []; // sample kara sagasu

    // case function has boolian flag "remove_above_threshold" = true
    const double velocity_threshold = 1.0; // eg 0.5
    bool remove_threshold = true; 

    expected_objects = [];
    ans = filterObjectsByVelocity(predicted_objects, velocity_threshold, remove_threshold);
    EXPECT_EQ(ans, expected_objects)


    // another case
    const double velocity_threshold = 1.0; // eg 0.5
    bool remove_threshold = false; 

    expected_objects = [];
    ans = filterObjectsByVelocity(predicted_objects, velocity_threshold, remove_threshold);
    EXPECT_EQ(ans, expected_objects)
}