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

#include "autoware_traffic_light_arbiter/traffic_light_arbiter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware_traffic_light_arbiter::TrafficLightArbiterNode;
using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
using TrafficSignal = autoware_perception_msgs::msg::TrafficLightGroup;
using TrafficElement = autoware_perception_msgs::msg::TrafficLightElement;


std::shared_ptr<autoware_test_utils::AutowareTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<autoware_test_utils::AutowareTestManager>();
  return test_manager;
}

std::shared_ptr<TrafficLightArbiterNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto package_dir = ament_index_cpp::get_package_share_directory("autoware_traffic_light_arbiter");
  node_options.arguments({"--ros-args", "--params-file", package_dir + "/config/traffic_light_arbiter.param.yaml"});
  return std:make_shared<TrafficLightArbiterNode>(node_options);
}

void generateMap(LaneletMapBin & vector_map_msg)
{
	const auto package_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
	lanelet::LaneletMapPtr vector_map_ptr = autoware_test_utils::loadMap(package_dir + "/test_map/lanelet2_map.osm");
	vector_map_msg.header.frame_id = "base_link";
	vector_map_msg.version = "1.0.0";
	vector_map_msg.name_map = "lanelet2_map";
	lanelet::utils::conversion::toBinMsg(*vector_map_ptr, &vector_map_msg);
}

bool isMsgEqual(const TrafficSignalArray & input_msg, const TrafficSignalArray & output_msg)
{
	for (const auto & input_traffic_light_group : input_msg.traffic_light_groups) {
		for (const auto & output_traffic_light_group : output_msg.traffic_light_groups) {
			if (input_traffic_light_group.traffic_light_group_id == output_traffic_light_group.traffic_light_group_id) {
				if (input_traffic_light_group.elements.size() != output_traffic_light_group.elements.size()) {
					return false;
				}
				for (size_t i = 0; i < input_traffic_light_group.elements.size(); i++) {
					if (input_traffic_light_group.elements[i].color != output_traffic_light_group.elements[i].color) {
						return false;
					}
					if (input_traffic_light_group.elements[i].shape != output_traffic_light_group.elements[i].shape) {
						return false;
					}
					if (input_traffic_light_group.elements[i].status != output_traffic_light_group.elements[i].status) {
						return false;
					}
					if (input_traffic_light_group.elements[i].confidence != output_traffic_light_group.elements[i].confidence) {
						return false;
					}
				}
			}
		}
	}
}


TEST(TrafficLightArbiterTest, testTrafficSignalOnlyPerception)
{
  rclcpp::init(0, nullptr);
  const std::string input_map_topic = "/sub/vector_map";
  const std::string input_perception_topic = "/sub/perception_traffic_signals";
  const std::string output_topic = "/pub/traffic_signals";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // Data preparation
	// Map
	LaneletMapBin vector_map_msg;
	generateMap(vector_map_msg);

	// Perception
	TrafficSignalArray perception_msg;
	{
		TrafficSignal traffic_light_groups;
		traffic_light_groups.traffic_light_group_id = 1;
		{
			TrafficElement elements;
			elements.color = TrafficElement::RED;
			elements.shape = TrafficElement::CIRCLE;
			elements.status = TrafficElement::SOLID_OFF;
			elements.confidence = 0.9;
			traffic_light_groups.elements.push_back(elements);
		}
		perception_msg.traffic_light_groups.push_back(traffic_light_groups);
	}

  // Test callback
  TrafficSignalArray latest_msg;
  auto callback = [&latest_msg](const TrafficSignalArray::SharedPtr msg) { latest_msg = *msg; };
  test_manager->set_subscriber<TrafficSignalArray>(output_topic, callback);

  test_manager->set_pub_msg<lanelet::LaneletMap>(test_target_node, input_map_topic, vector_map_msg);
  test_manager->set_pub_msg<TrafficSignalArray>(test_target_node, input_perception_topic, perception_msg);

	EXPECT_EQ(isMsgEqual(perception_msg, latest_msg), true);
  rclcpp::shutdown();
}