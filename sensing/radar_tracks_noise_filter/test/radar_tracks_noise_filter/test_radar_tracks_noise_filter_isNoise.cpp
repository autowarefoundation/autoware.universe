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


#include "radar_tracks_noise_filter/radar_tracks_noise_filter_node.hpp"

#include <radar_msgs/msg/radar_scan.hpp>

#include <gtest/gtest.h>


std::shared_ptr<radar_tracks_noise_filter::RadarTrackCrossingNoiseFilterNode> get_node(float velocity_y_threshold)
{
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({
      {"node_params.is_amplitude_filter", true},
      {"velocity_y_threshold", velocity_y_threshold},
  });

  auto node = std::make_shared<radar_tracks_noise_filter::RadarTrackCrossingNoiseFilterNode>(node_options);
  return node;
}

radar_msgs::msg::RadarTrack get_radar_track(float velocity_y)
{
  radar_msgs::msg::RadarTrack radar_track;
  radar_track.velocity.y = velocity_y;  // 修正: 変数名を radar_track に修正
  return radar_track;
}

TEST(RadarTracksNoiseFilter, isNoise)
{
  using radar_tracks_noise_filter::RadarTrackCrossingNoiseFilterNode;
  using radar_msgs::msg::RadarTrack;
  rclcpp::init(0, nullptr);
  {
    float velnode_thr = 0.0;
    float radar_track_vel = 0.0;
    std::shared_ptr<RadarTrackCrossingNoiseFilterNode> node = get_node(velnode_thr);
    RadarTrack radar_track = get_radar_track(radar_track_vel);
    EXPECT_TRUE(node->isNoise(radar_track));
  }

  {
    float velnode_thr = -1.0;
    float radar_track_vel = 3.0;
    std::shared_ptr<RadarTrackCrossingNoiseFilterNode> node = get_node(velnode_thr);
    RadarTrack radar_track = get_radar_track(radar_track_vel);
    EXPECT_TRUE(node->isNoise(radar_track));
  }

  {
    float velnode_thr = 3.0;
    float radar_track_vel = 1.0;
    std::shared_ptr<RadarTrackCrossingNoiseFilterNode> node = get_node(velnode_thr);
    RadarTrack radar_track = get_radar_track(radar_track_vel);
    EXPECT_FALSE(node->isNoise(radar_track));
  }
}
