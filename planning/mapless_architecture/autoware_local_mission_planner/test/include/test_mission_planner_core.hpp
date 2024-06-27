// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
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
