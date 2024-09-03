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

#include "autoware/traffic_light_arbiter/traffic_light_arbiter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware::TrafficLightArbiter;
using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
using TrafficSignal = autoware_perception_msgs::msg::TrafficLightGroup;
using TrafficElement = autoware_perception_msgs::msg::TrafficLightElement;


std::shared_ptr<autoware::test_utils::AutowareTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<autoware::test_utils::AutowareTestManager>();
  return test_manager;
}

std::shared_ptr<TrafficLightArbiter> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto package_dir = ament_index_cpp::get_package_share_directory("autoware_traffic_light_arbiter");
  node_options.arguments({"--ros-args", "--params-file", package_dir + "/config/traffic_light_arbiter.param.yaml"});
  return std::make_shared<TrafficLightArbiter>(node_options);
}

void generateMap(LaneletMapBin & vector_map_msg)
{
	const auto package_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
	lanelet::LaneletMapPtr vector_map_ptr = autoware::test_utils::loadMap(package_dir + "/test_map/lanelet2_map.osm");
	vector_map_msg.header.frame_id = "base_link";
	lanelet::utils::conversion::toBinMsg(vector_map_ptr, &vector_map_msg);
}

bool isTrafficLightElementEqual(const TrafficElement & input_traffic_light_element, const TrafficElement & output_traffic_light_element)
{
	if (input_traffic_light_element.color != output_traffic_light_element.color) {
		return false;
	}
	if (input_traffic_light_element.shape != output_traffic_light_element.shape) {
		return false;
	}
	if (input_traffic_light_element.status != output_traffic_light_element.status) {
		return false;
	}
	if (input_traffic_light_element.confidence != output_traffic_light_element.confidence) {
		return false;
	}
	return true;
}

bool isTrafficLightGroupEqual(const TrafficSignal & input_traffic_light_group, const TrafficSignal & output_traffic_light_group)
{
	if (input_traffic_light_group.traffic_light_group_id != output_traffic_light_group.traffic_light_group_id) {
		return false;
	}
	if (input_traffic_light_group.elements.size() != output_traffic_light_group.elements.size()) {
		return false;
	}
	for (const auto & input_traffic_light_element : input_traffic_light_group.elements) {
		for (const auto & output_traffic_light_element : output_traffic_light_group.elements) {
			if (!isTrafficLightElementEqual(input_traffic_light_element, output_traffic_light_element)) {
				return false;
			}
		}
	}
	return true;
}

bool isMsgEqual(const TrafficSignalArray & input_msg, const TrafficSignalArray & output_msg)
{
	if (input_msg.traffic_light_groups.size() != output_msg.traffic_light_groups.size()) {
		return false;
	}
	for (const auto & input_traffic_light_group : input_msg.traffic_light_groups) {
		for (const auto & output_traffic_light_group : output_msg.traffic_light_groups) {
			if (!isTrafficLightGroupEqual(input_traffic_light_group, output_traffic_light_group)) {
				return false;
			}
		}
	}
	return true;
}

TEST(TrafficLightArbiterTest, testTrafficSignalOnlyPerception)
{
  rclcpp::init(0, nullptr);
  const std::string input_map_topic = "/traffic_light_arbiter/sub/vector_map";
  const std::string input_perception_topic = "/traffic_light_arbiter/sub/perception_traffic_signals";
  const std::string output_topic = "/traffic_light_arbiter/pub/traffic_signals";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // map preparation
	LaneletMapBin vector_map_msg;
	generateMap(vector_map_msg);

	// perception preparation
	TrafficSignalArray perception_msg;
	{
		TrafficSignal traffic_light_groups;
		traffic_light_groups.traffic_light_group_id = 1012;
		{
			TrafficElement elements;
			elements.color = TrafficElement::RED;
			elements.shape = TrafficElement::CIRCLE;
			elements.status = TrafficElement::SOLID_OFF;
			elements.confidence = 1.0;
			traffic_light_groups.elements.push_back(elements);
		}
		perception_msg.traffic_light_groups.push_back(traffic_light_groups);
	}

  // test callback
  TrafficSignalArray latest_msg;
  auto callback = [&latest_msg](const TrafficSignalArray::ConstSharedPtr msg) { latest_msg = *msg; };
  test_manager->set_subscriber<TrafficSignalArray>(output_topic, callback);

  test_manager->test_pub_msg<LaneletMapBin>(test_target_node, input_map_topic, vector_map_msg, rclcpp::QoS(1).transient_local());
	test_manager->test_pub_msg<TrafficSignalArray>(test_target_node, input_perception_topic, perception_msg);

	EXPECT_EQ(isMsgEqual(perception_msg, latest_msg), true);
  rclcpp::shutdown();
}

TEST(TrafficLightArbiterTest, testTrafficSignalBoth)
{
  rclcpp::init(0, nullptr);
  const std::string input_map_topic = "/traffic_light_arbiter/sub/vector_map";
  const std::string input_perception_topic = "/traffic_light_arbiter/sub/perception_traffic_signals";
	const std::string input_external_topic = "/traffic_light_arbiter/sub/external_traffic_signals";
  const std::string output_topic = "/traffic_light_arbiter/pub/traffic_signals";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // map preparation
	LaneletMapBin vector_map_msg;
	generateMap(vector_map_msg);

	// perception preparation
	TrafficSignalArray perception_msg;
	{
		TrafficSignal traffic_light_groups;
		traffic_light_groups.traffic_light_group_id = 1012;
		{
			TrafficElement elements;
			elements.color = TrafficElement::RED;
			elements.shape = TrafficElement::CIRCLE;
			elements.status = TrafficElement::SOLID_ON;
			elements.confidence = 1.0;
			traffic_light_groups.elements.push_back(elements);
		}
		perception_msg.traffic_light_groups.push_back(traffic_light_groups);
	}

	// external preparation
	TrafficSignalArray external_msg;
	{
		TrafficSignal traffic_light_groups;
		traffic_light_groups.traffic_light_group_id = 1012;
		{
			TrafficElement elements;
			elements.color = TrafficElement::GREEN;
			elements.shape = TrafficElement::CIRCLE;
			elements.status = TrafficElement::SOLID_ON;
			elements.confidence = 1.0;
			traffic_light_groups.elements.push_back(elements);
		}
		external_msg.traffic_light_groups.push_back(traffic_light_groups);
	}

  // test callback
  TrafficSignalArray latest_msg;
  auto callback = [&latest_msg](const TrafficSignalArray::ConstSharedPtr msg) { latest_msg = *msg; };
  test_manager->set_subscriber<TrafficSignalArray>(output_topic, callback);

  test_manager->test_pub_msg<LaneletMapBin>(test_target_node, input_map_topic, vector_map_msg, rclcpp::QoS(1).transient_local());
	test_manager->test_pub_msg<TrafficSignalArray>(test_target_node, input_external_topic, external_msg);
	test_manager->test_pub_msg<TrafficSignalArray>(test_target_node, input_perception_topic, perception_msg);

	EXPECT_EQ(isMsgEqual(perception_msg, latest_msg), true);
  rclcpp::shutdown();
}
