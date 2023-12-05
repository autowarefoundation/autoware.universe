// Copyright 2023 Tier IV, Inc.
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

#include "radar_threshold_filter/radar_threshold_filter_node.hpp"

#include <gtest/gtest.h>

#include <radar_msgs/msg/radar_scan.hpp>


TEST(RadarThresholdFilter, isWithinThreshold){
  rclcpp::init(0, nullptr);
  using radar_threshold_filter::RadarThresholdFilterNode;
  using radar_msgs::msg::RadarReturn;
  // amplitude filter
  {
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(
    {
      {"node_params.is_amplitude_filter", true},
      {"node_params.amplitude_min", 0.0},
      {"node_params.amplitude_max", 10.0},
    });

    RadarThresholdFilterNode node(node_options);
    RadarReturn radar_return;

    // Now you can use radar_return and set its fields as needed
    radar_return.amplitude = 5.0;
    EXPECT_TRUE(node.isWithinThreshold(radar_return));

    radar_return.amplitude = -1.0;
    EXPECT_FALSE(node.isWithinThreshold(radar_return));
    }

    // // range filter
    // {
    //   rclcpp::NodeOptions node_options;
    //   node_options.parameter_overrides(
    //   {
    //     {"node_params.is_range_filter", true},
    //     {"node_params.range_min", 0.0},
    //     {"node_params.range_max", 10.0},
    //   });

    //   RadarThresholdFilterNode node(node_options);
    //   RadarReturn radar_return;
    //   radar_return.range = 5.0;
    //   EXPECT_TRUE(node.isWithinThreshold(radar_return));
    //   radar_return.range = -1.0;
    //   EXPECT_FALSE(node.isWithinThreshold(radar_return));
    //   radar_return.range = 0.0;
    //   EXPECT_FALSE(node.isWithinThreshold(radar_return));
    // }

    // // azimuth filter
    // {
    //   rclcpp::NodeOptions node_options;
    //   node_options.parameter_overrides(
    //   {
    //     {"node_params.is_azimuth_filter", true},
    //     {"node_params.azimuth_min", 0.0},
    //     {"node_params.azimuth_max", 10.0},
    //   });

    //   RadarThresholdFilterNode node(node_options);
    //   RadarReturn radar_return;
    //   radar_return.azimuth = 5.0;
    //   EXPECT_TRUE(node.isWithinThreshold(radar_return));
    //   radar_return.azimuth = -1.0;
    //   EXPECT_FALSE(node.isWithinThreshold(radar_return));
    //   radar_return.azimuth = 0.0;
    //   EXPECT_FALSE(node.isWithinThreshold(radar_return));
    // }

    // // is_azimuth_filter
    // {
    //   rclcpp::NodeOptions node_options;
    //   node_options.parameter_overrides(
    //   {
    //     {"node_params.is_z_filter", true},
    //     {"node_params.z_min", 0.0},
    //     {"node_params.z_max", 10.0},
    //   });

    //   RadarThresholdFilterNode node(node_options);
    //   RadarReturn radar_return;
    //   radar_return.elevation = M_PI/2;
    //   // z / std::sin(radar_return.elevation);
    //   radar_return.range = 5.0;

    //   EXPECT_TRUE(node.isWithinThreshold(radar_return));
    //   radar_return.range = -1.0;
    //   EXPECT_FALSE(node.isWithinThreshold(radar_return));
    //   radar_return.range = 0.0;
    //   EXPECT_FALSE(node.isWithinThreshold(radar_return));
    // }
}

