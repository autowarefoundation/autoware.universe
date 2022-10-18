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

constexpr double epsilon = 1e-06;

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
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    std::uint8_t label = getHighestProbLabel(classifications);
    EXPECT_EQ(label, ObjectClassification::UNKNOWN);
  }

  {  // normal case
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    classifications.push_back(createObjectClassification(ObjectClassification::CAR, 0.5));
    classifications.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    std::uint8_t label = getHighestProbLabel(classifications);
    EXPECT_EQ(label, ObjectClassification::TRUCK);
  }

  {  // labels with the same probability
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    classifications.push_back(createObjectClassification(ObjectClassification::CAR, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    std::uint8_t label = getHighestProbLabel(classifications);
    EXPECT_EQ(label, ObjectClassification::CAR);
  }
}

TEST(object_classification, test_getHighestProbClassification)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::getHighestProbClassification;

  {  // empty
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    auto classification = getHighestProbClassification(classifications);
    EXPECT_EQ(classification.label, ObjectClassification::UNKNOWN);
    EXPECT_DOUBLE_EQ(classification.probability, 0.0);
  }

  {  // normal case
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    classifications.push_back(createObjectClassification(ObjectClassification::CAR, 0.5));
    classifications.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    auto classification = getHighestProbClassification(classifications);
    EXPECT_EQ(classification.label, ObjectClassification::TRUCK);
    EXPECT_NEAR(classification.probability, 0.8, epsilon);
  }

  {  // labels with the same probability
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    classifications.push_back(createObjectClassification(ObjectClassification::CAR, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    auto classification = getHighestProbClassification(classifications);
    EXPECT_EQ(classification.label, ObjectClassification::CAR);
    EXPECT_NEAR(classification.probability, 0.8, epsilon);
  }
}

TEST(object_classification, test_fromString)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::toLabel;
  using perception_utils::toObjectClassification;
  using perception_utils::toObjectClassifications;

  // toLabel
  {
    EXPECT_EQ(toLabel("UNKNOWN"), ObjectClassification::UNKNOWN);
    EXPECT_EQ(toLabel("CAR"), ObjectClassification::CAR);
    EXPECT_EQ(toLabel("TRUCK"), ObjectClassification::TRUCK);
    EXPECT_EQ(toLabel("BUS"), ObjectClassification::BUS);
    EXPECT_EQ(toLabel("TRAILER"), ObjectClassification::TRAILER);
    EXPECT_EQ(toLabel("MOTORCYCLE"), ObjectClassification::MOTORCYCLE);
    EXPECT_EQ(toLabel("BICYCLE"), ObjectClassification::BICYCLE);
    EXPECT_EQ(toLabel("PEDESTRIAN"), ObjectClassification::PEDESTRIAN);
    EXPECT_EQ(toLabel(""), ObjectClassification::UNKNOWN);
  }

  // Classification
  {
    auto classification = toObjectClassification("CAR", 0.7);
    EXPECT_EQ(classification.label, ObjectClassification::CAR);
    EXPECT_NEAR(classification.probability, 0.7, epsilon);
  }
  // Classifications
  {
    auto classifications = toObjectClassifications("CAR", 0.7);
    EXPECT_EQ(classifications.at(0).label, ObjectClassification::CAR);
    EXPECT_NEAR(classifications.at(0).probability, 0.7, epsilon);
  }
}

TEST(object_classification, test_toString)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using perception_utils::toString;

  // from label
  {
    EXPECT_EQ(toString(ObjectClassification::UNKNOWN), "UNKNOWN");
    EXPECT_EQ(toString(ObjectClassification::CAR), "CAR");
    EXPECT_EQ(toString(ObjectClassification::TRUCK), "TRUCK");
    EXPECT_EQ(toString(ObjectClassification::BUS), "BUS");
    EXPECT_EQ(toString(ObjectClassification::TRAILER), "TRAILER");
    EXPECT_EQ(toString(ObjectClassification::MOTORCYCLE), "MOTORCYCLE");
    EXPECT_EQ(toString(ObjectClassification::BICYCLE), "BICYCLE");
    EXPECT_EQ(toString(ObjectClassification::PEDESTRIAN), "PEDESTRIAN");
  }

  // from ObjectClassification
  {
    auto classification = createObjectClassification(ObjectClassification::CAR, 0.8);

    EXPECT_EQ(toString(classification), "CAR");
  }

  // from ObjectClassifications
  {
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classifications;
    classifications.push_back(createObjectClassification(ObjectClassification::CAR, 0.5));
    classifications.push_back(createObjectClassification(ObjectClassification::TRUCK, 0.8));
    classifications.push_back(createObjectClassification(ObjectClassification::BUS, 0.7));

    EXPECT_EQ(toString(classifications), "TRUCK");
  }
}
