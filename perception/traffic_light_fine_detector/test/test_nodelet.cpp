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

#include "traffic_light_fine_detector/nodelet.hpp"

#include <gtest/gtest.h>

sensor_msgs::msg::RegionOfInterest createMapBasedBbox(
  const uint32_t x_offset, const uint32_t y_offset, const uint32_t width, const uint32_t height)
{
  sensor_msgs::msg::RegionOfInterest bbox;
  bbox.x_offset = x_offset;
  bbox.y_offset = y_offset;
  bbox.width = width;
  bbox.height = height;
  return bbox;
}

tensorrt_yolox::Object createYoloxBbox(
  const int32_t x_offset, const int32_t y_offset, const int32_t width, const int32_t height,
  const float score, const int32_t type)
{
  tensorrt_yolox::Object bbox;
  bbox.x_offset = x_offset;
  bbox.y_offset = y_offset;
  bbox.width = width;
  bbox.height = height;
  bbox.score = score;
  bbox.type = type;
  return bbox;
}

TEST(CalWeightedIouTest, NoOverlap)
{
  const sensor_msgs::msg::RegionOfInterest map_based_bbox = createMapBasedBbox(0, 0, 10, 10);
  const tensorrt_yolox::Object yolox_bbox = createYoloxBbox(20, 20, 10, 10, 0.9f, 0);

  EXPECT_FLOAT_EQ(traffic_light::calWeightedIou(map_based_bbox, yolox_bbox), 0.0f);
}

TEST(CalWeightedIouTest, PartiallyOverlap1)
{
  const sensor_msgs::msg::RegionOfInterest map_based_bbox = createMapBasedBbox(15, 15, 10, 10);
  const tensorrt_yolox::Object yolox_bbox = createYoloxBbox(20, 20, 10, 10, 0.7f, 0);

  EXPECT_FLOAT_EQ(traffic_light::calWeightedIou(map_based_bbox, yolox_bbox), 0.1f);
}

TEST(CalWeightedIouTest, PartiallyOverlap2)
{
  const sensor_msgs::msg::RegionOfInterest map_based_bbox = createMapBasedBbox(0, 0, 10, 10);
  const tensorrt_yolox::Object yolox_bbox = createYoloxBbox(-5, -5, 10, 10, 0.7f, 0);

  EXPECT_FLOAT_EQ(traffic_light::calWeightedIou(map_based_bbox, yolox_bbox), 0.1f);
}

TEST(CalWeightedIouTest, Included1)
{
  const sensor_msgs::msg::RegionOfInterest map_based_bbox = createMapBasedBbox(20, 20, 10, 10);
  const tensorrt_yolox::Object yolox_bbox = createYoloxBbox(20, 20, 10, 10, 0.5f, 0);

  EXPECT_FLOAT_EQ(traffic_light::calWeightedIou(map_based_bbox, yolox_bbox), 0.5f);
}

TEST(CalWeightedIouTest, Included2)
{
  const sensor_msgs::msg::RegionOfInterest map_based_bbox = createMapBasedBbox(20, 20, 100, 100);
  const tensorrt_yolox::Object yolox_bbox = createYoloxBbox(20, 20, 10, 10, 0.5f, 0);

  EXPECT_FLOAT_EQ(traffic_light::calWeightedIou(map_based_bbox, yolox_bbox), 0.005f);
}

TEST(CalWeightedIouTest, Zero)
{
  const sensor_msgs::msg::RegionOfInterest map_based_bbox = createMapBasedBbox(0, 0, 0, 0);
  const tensorrt_yolox::Object yolox_bbox = createYoloxBbox(0, 0, 0, 0, 0.5f, 0);

  EXPECT_FLOAT_EQ(traffic_light::calWeightedIou(map_based_bbox, yolox_bbox), 0.0f);
}

TEST(CalWeightedIouTest, Negative)
{
  const sensor_msgs::msg::RegionOfInterest map_based_bbox = createMapBasedBbox(0, 0, 10, 10);
  const tensorrt_yolox::Object yolox_bbox = createYoloxBbox(-5, -5, -5, 10, 0.5f, 0);

  EXPECT_FLOAT_EQ(traffic_light::calWeightedIou(map_based_bbox, yolox_bbox), 0.0f);
}

// This test case should be passed but it fails because of inappropriate type casting.
// So it is disabled now. Current result is 0.1097561, but it should be 0.0.
TEST(CalWeightedIouTest, DISABLED_Uint32Max)
{
  const sensor_msgs::msg::RegionOfInterest map_based_bbox =
    createMapBasedBbox(UINT32_MAX, UINT32_MAX, 10, 10);
  const tensorrt_yolox::Object yolox_bbox = createYoloxBbox(-5, -5, 10, 10, 0.5f, 0);

  EXPECT_FLOAT_EQ(traffic_light::calWeightedIou(map_based_bbox, yolox_bbox), 0.0f);
}
