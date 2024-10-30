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

#include "utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/detail/object_classification__struct.hpp>
#include <tier4_planning_msgs/msg/detail/path_point_with_lane_id__struct.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

using autoware::behavior_velocity_planner::run_out_utils::createBoostPolyFromMsg;
using autoware::behavior_velocity_planner::run_out_utils::getHighestProbLabel;
using autoware_perception_msgs::msg::ObjectClassification;
using geometry_msgs::msg::Point;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;

class TestRunOutUtils : public ::testing::Test
{
  void SetUp() override {}
};
namespace autoware::behavior_velocity_planner::run_out_utils
{
/**
 * @brief Test the external command converter's check_emergency_stop_topic_timeout method under
 * different circumstances by setting different combinations of conditions for the function
 * execution. the test_check_emergency_stop_topic_timeout() function's brief should be read for
 * further details.
 */
TEST_F(TestRunOutUtils, testFindLongitudinalNearestPoint)
{
  std::vector<geometry_msgs::msg::Point> poly;
  poly.push_back(autoware::universe_utils::createPoint(0.0, 1.0, 0.0));
  poly.push_back(autoware::universe_utils::createPoint(0.0, -1.0, 0.0));
  poly.push_back(autoware::universe_utils::createPoint(1.0, 1.0, 0.0));
  poly.push_back(autoware::universe_utils::createPoint(1.0, 0.0, 0.0));

  const auto boost_poly = createBoostPolyFromMsg(poly);
  EXPECT_FALSE(boost_poly.outer().empty());
  EXPECT_EQ(boost_poly.outer().size(), poly.size() + 1);
}

TEST_F(TestRunOutUtils, testGetHighestProbLabel)
{
  ObjectClassification classification_1;
  classification_1.label = ObjectClassification::BICYCLE;
  classification_1.probability = 0.7;

  ObjectClassification classification_2;
  classification_2.label = ObjectClassification::MOTORCYCLE;
  classification_2.probability = 0.1;

  ObjectClassification classification_3;
  classification_3.label = ObjectClassification::TRUCK;
  classification_3.probability = 0.2;

  std::vector<ObjectClassification> classifications{
    classification_1, classification_2, classification_3};
  const auto classification = getHighestProbLabel(classifications);

  EXPECT_EQ(classification, classification_1.label);
}

TEST_F(TestRunOutUtils, testGetHighestConfidencePath)
{
  std::vector<PredictedPath> predicted_paths{};
  const auto empty_path = getHighestConfidencePath(predicted_paths);
  EXPECT_TRUE(empty_path.empty());
  geometry_msgs::msg::Pose p1;
  p1.position.x = 1.0;

  geometry_msgs::msg::Pose p2;
  p1.position.x = 2.0;

  PredictedPath predicted_path_1{};
  predicted_path_1.path = {p1};
  predicted_path_1.confidence = 0.85;

  PredictedPath predicted_path_2{};
  predicted_path_2.path = {p2};
  predicted_path_2.confidence = 0.15;

  predicted_paths.push_back(predicted_path_1);
  predicted_paths.push_back(predicted_path_2);

  const auto high_confidence_path = getHighestConfidencePath(predicted_paths);
  const auto path_point = high_confidence_path.front();
  EXPECT_DOUBLE_EQ(0.0, autoware::universe_utils::calcDistance3d(path_point, p1));
}
}  // namespace autoware::behavior_velocity_planner::run_out_utils
