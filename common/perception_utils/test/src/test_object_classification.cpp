// Copyright 2022 TIER IV, Inc.
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

#include "perception_utils/object_classification.hpp"

#include <gtest/gtest.h>

namespace
{
autoware_auto_perception_msgs::msg::ObjectClassification createObjectClassification(
  const std::uint8_t label, const double probability)
{
  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  classification.label = label;
  classification.probability = probability;

  return classification;
}

}  // namespace

TEST(object_classification, test_getHighestProbLabel)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::getHighestProbLabel;

  {  // empty
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
    std::uint8_t label = getHighestProbLabel(classification);
    EXPECT_EQ(label, ObjectClassification::UNKNOWN);
  }

  {  // normal case
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
    classification.push_back(createObjectClassification(ObjectClassification::CAR, 0.5));
    classification.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    std::uint8_t label = getHighestProbLabel(classification);
    EXPECT_EQ(label, ObjectClassification::TRUCK);
  }

  {  // labels with the same probability
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
    classification.push_back(createObjectClassification(ObjectClassification::CAR, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    std::uint8_t label = getHighestProbLabel(classification);
    EXPECT_EQ(label, ObjectClassification::CAR);
  }
}

// Test isVehicle
TEST(object_classification, test_isVehicle)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::isVehicle;

  {// True Case with uint8_t
  EXPECT_TRUE(isVehicle(ObjectClassification::BICYCLE));
  EXPECT_TRUE(isVehicle(ObjectClassification::BUS));
  EXPECT_TRUE(isVehicle(ObjectClassification::CAR));
  EXPECT_TRUE(isVehicle(ObjectClassification::MOTORCYCLE));
  EXPECT_TRUE(isVehicle(ObjectClassification::TRAILER));
  EXPECT_TRUE(isVehicle(ObjectClassification::TRUCK));
  }

  // False Case with uint8_t
  {
  EXPECT_FALSE(isVehicle(ObjectClassification::UNKNOWN));
  EXPECT_FALSE(isVehicle(ObjectClassification::PEDESTRIAN));
  }

  // True Case with object_classifications
  {  // normal case
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
    classification.push_back(createObjectClassification(ObjectClassification::CAR, 0.5));
    classification.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));
    EXPECT_TRUE(isVehicle(classification));
  }

  // False Case with object_classifications
  {  // false case
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
    classification.push_back(createObjectClassification(ObjectClassification::PEDESTRIAN, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::BICYCLE, 0.7));
    EXPECT_FALSE(isVehicle(classification));
  }

}  // TEST isVehicle

// TEST isCarLikeVehicle
TEST(object_classification, test_isCarLikeVehicle)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::isCarLikeVehicle;

  {// True Case with uint8_t
  EXPECT_TRUE(isCarLikeVehicle(ObjectClassification::BUS));
  EXPECT_TRUE(isCarLikeVehicle(ObjectClassification::CAR));
  EXPECT_TRUE(isCarLikeVehicle(ObjectClassification::TRAILER));
  EXPECT_TRUE(isCarLikeVehicle(ObjectClassification::TRUCK));  
  }

  // False Case with uint8_t
  {
  EXPECT_FALSE(isCarLikeVehicle(ObjectClassification::UNKNOWN));
  EXPECT_FALSE(isCarLikeVehicle(ObjectClassification::BICYCLE));
  EXPECT_FALSE(isCarLikeVehicle(ObjectClassification::PEDESTRIAN));
  EXPECT_FALSE(isCarLikeVehicle(ObjectClassification::MOTORCYCLE));
  }

  // True Case with object_classifications
  {  // normal case
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
    classification.push_back(createObjectClassification(ObjectClassification::CAR, 0.5));
    classification.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::BICYCLE, 0.7));
    EXPECT_TRUE(isCarLikeVehicle(classification));
  }

  // False Case with object_classifications
  {  // false case
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
    classification.push_back(createObjectClassification(ObjectClassification::MOTORCYCLE, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::BICYCLE, 0.8));
    EXPECT_FALSE(isCarLikeVehicle(classification));
  }

}  // TEST isCarLikeVehicle

// TEST isLargeVehicle
TEST(object_classification, test_isLargeVehicle)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::isLargeVehicle;

  {// True Case with uint8_t
  EXPECT_TRUE(isLargeVehicle(ObjectClassification::BUS));
  EXPECT_TRUE(isLargeVehicle(ObjectClassification::TRAILER));
  EXPECT_TRUE(isLargeVehicle(ObjectClassification::TRUCK));
  }

  // False Case with uint8_t
  {
  EXPECT_FALSE(isLargeVehicle(ObjectClassification::UNKNOWN));
  EXPECT_FALSE(isLargeVehicle(ObjectClassification::BICYCLE));
  EXPECT_FALSE(isLargeVehicle(ObjectClassification::PEDESTRIAN));
  EXPECT_FALSE(isLargeVehicle(ObjectClassification::MOTORCYCLE));
  EXPECT_FALSE(isLargeVehicle(ObjectClassification::CAR));
  }

  {  // false case
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
    classification.push_back(createObjectClassification(ObjectClassification::MOTORCYCLE, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::BICYCLE, 0.8));
    classification.push_back(createObjectClassification(ObjectClassification::CAR, 0.8));
    EXPECT_FALSE(isLargeVehicle(classification));
  }

}  // TEST isLargeVehicle
