// Copyright 2024 driveblocks GmbH, authors: Simon Eisenmann, Thomas Herrmann
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_MISSION_PLANNER_CORE_HPP_
#define TEST_MISSION_PLANNER_CORE_HPP_

namespace autoware::mapless_architecture
{

/**
 * @brief Test distance between point and LineString calculation.
 *
 * This function tests CalculateDistanceBetweenPointAndLineString_() which
 * calculates the distance between a point and a LineString.
 *
 * @return int: returns 0 on success
 */
int TestCalculateDistanceBetweenPointAndLineString();

/**
 * @brief Test GetPointOnLane_() function.
 *
 * This function tests GetPointOnLane_() which
 * returns a point on a given lane in a given distance (x axis).
 *
 * @return int: returns 0 on success
 */
int TestGetPointOnLane();

/**
 * @brief Test RecenterGoalPoint() function.
 *
 * @return int: returns 0 on success
 */
int TestRecenterGoalpoint();

/**
 * @brief Test IsOnGoalLane_() function.
 *
 * This function tests IsOnGoalLane_() which
 * returns true if the vehicle is on the goal lane (defined by the goal point).
 *
 * @return int: returns 0 on success
 */
int TestIsOnGoalLane();

/**
 * @brief Test CheckIfGoalPointShouldBeReset_() function.
 *
 * If the x value of the goal point is negative, the goal point should be reset.
 *
 * @return int: returns 0 on success
 */
int TestCheckIfGoalPointShouldBeReset();

/**
 * @brief Test CalculateLanes_() function.
 *
 * @return int: returns 0 on success
 */
int TestCalculateLanes();

/**
 * @brief Test CreateMarkerArray_() function.
 *
 * @return int: returns 0 on success
 */
int TestCreateMarkerArray();

/**
 * @brief Test CreateDrivingCorridor_() function.
 *
 * @return int: returns 0 on success
 */
int TestCreateDrivingCorridor();

}  // namespace autoware::mapless_architecture

#endif  // TEST_MISSION_PLANNER_CORE_HPP_
