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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>

#include <yaml-cpp/yaml.h>

#include <array>
#include <filesystem>
#include <mutex>
#include <optional>
#include <regex>
#include <unordered_map>
#include <variant>

using MessageType = std::variant<
  nav_msgs::msg::Odometry,                               // 0
  geometry_msgs::msg::AccelWithCovarianceStamped,        // 1
  autoware_perception_msgs::msg::PredictedObjects,       // 2
  autoware_adapi_v1_msgs::msg::OperationModeState,       // 3
  autoware_planning_msgs::msg::LaneletRoute,             // 4
  autoware_perception_msgs::msg::TrafficLightGroupArray  // 5
  >;

std::optional<size_t> get_topic_index(const std::string & name)
{
  if (name == "Odometry") {
    return 0;
  }
  if (name == "AccelWithCovarianceStamped") {
    return 1;
  }
  if (name == "PredictedObjects") {
    return 2;
  }
  if (name == "OperationModeState") {
    return 3;
  }
  if (name == "LaneletRoute") {
    return 4;
  }
  if (name == "TrafficLightGroupArray") {
    return 5;
  }
  return std::nullopt;
}

template <size_t TypeIndex, typename Callback>
typename rclcpp::SubscriptionBase::SharedPtr create_subscriber(
  const std::string & topic_name, rclcpp::Node & node, Callback && callback)
{
  return node.create_subscription<typename std::variant_alternative_t<TypeIndex, MessageType>>(
    topic_name, 1, std::forward<Callback>(callback));
}

template <typename Message>
class CallbackHandler
{
public:
  CallbackHandler(std::mutex & m, MessageType & buffer) : mutex_(m), buffer_(buffer) {}

  // nav_msgs::msg::Odometry_<std::allocator<void>>::SharedPtr
  void on_callback(const typename Message::SharedPtr msg)
  {
    std::lock_guard guard(mutex_);
    buffer_ = *msg;
  }

private:
  std::mutex & mutex_;
  MessageType & buffer_;
};

std::optional<std::string> resolve_pkg_share_uri(const std::string & uri_path)
{
  std::smatch match;
  std::regex pattern(R"(package://([^/]+)/(.+))");
  if (std::regex_match(uri_path, match, pattern)) {
    const std::string pkg_name = ament_index_cpp::get_package_share_directory(match[1].str());
    const std::string resource_path = match[2].str();
    const auto path = std::filesystem::path(pkg_name) / std::filesystem::path(resource_path);
    return std::filesystem::exists(path) ? std::make_optional<std::string>(path) : std::nullopt;
  }
  return std::nullopt;
}

class TopicSnapShotSaver
{
public:
  TopicSnapShotSaver(
    const std::string & map_path, const std::string & config_path, rclcpp::Node & node)
  : map_path_(map_path), config_path_(config_path)
  {
    server_ = node.create_service<std_srvs::srv::Empty>(
      "/autoware_test_utils/topic_snapshot_saver",
      std::bind(
        &TopicSnapShotSaver::on_service, this, std::placeholders::_1, std::placeholders::_2));

    // get map hash
    // setup subscribers and callbacks
    const auto config = YAML::LoadFile(config_path);
    for (const auto field : config["fields"]) {
      const auto name = field["name"].as<std::string>();
      const auto type_index_opt = get_topic_index(field["type"].as<std::string>());
      if (!type_index_opt) {
        continue;
      }
      const auto type_index = type_index_opt.value();
      const auto topic = field["topic"].as<std::string>();
      if (type_index == 0) {
        CallbackHandler<typename std::variant_alternative_t<0, MessageType>> handler(
          mutex_, message_buffer_.at(0));
        const auto sub = create_subscriber<0>(
          topic, node,
          std::bind(
            &CallbackHandler<std::variant_alternative_t<0, MessageType>>::on_callback, &handler,
            std::placeholders::_1));
      }
    }
  }

  void on_callback([[maybe_unused]] const nav_msgs::msg::Odometry::SharedPtr msg) { return; }

  const std::string map_path_;
  const std::string config_path_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_;
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscribers_;

  std::array<MessageType, std::variant_size_v<MessageType>> message_buffer_;
  std::mutex mutex_;

  void on_service(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
  }
};

class TopicSnapShotSaverFrontEnd : public rclcpp::Node
{
public:
  TopicSnapShotSaverFrontEnd() : Node("topic_snapshot_saver_frontned")
  {
    const auto map_path_uri = declare_parameter<std::string>("map_path", "none");
    const auto config_path_uri = declare_parameter<std::string>("config_path", "none");
    if (map_path_uri == "none" || config_path_uri == "none") {
      return;
    }
    const auto map_path = resolve_pkg_share_uri(map_path_uri);
    const auto config_path = resolve_pkg_share_uri(config_path_uri);
    if (!map_path) {
      RCLCPP_ERROR(
        get_logger(),
        "failed to resolve %s. expected form is package://<package-name>/<resouce-path>",
        map_path_uri.c_str());
    } else if (!config_path) {
      RCLCPP_ERROR(
        get_logger(),
        "failed to resolve %s. expected form is package://<package-name>/<resouce-path>",
        config_path_uri.c_str());
    } else {
      snap_shot_saver_ =
        std::make_shared<TopicSnapShotSaver>(map_path.value(), config_path.value(), *this);
    }
  }

private:
  std::shared_ptr<TopicSnapShotSaver> snap_shot_saver_{nullptr};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
}
