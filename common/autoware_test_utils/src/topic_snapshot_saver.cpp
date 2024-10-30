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
#include <autoware_test_utils/mock_data_parser.hpp>
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
#include <fstream>
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

std::mutex g_mutex;

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
  CallbackHandler(MessageType & buffer) : buffer_(buffer) {}

  void on_callback(const typename Message::SharedPtr msg)
  {
    std::lock_guard guard(g_mutex);
    if (msg) {
      buffer_ = *msg;
    }
  }

private:
  MessageType & buffer_;
};

namespace detail
{
template <typename T>
struct CallbackHandlerType;

template <typename... Ts>
struct CallbackHandlerType<std::variant<Ts...>>
{
  using type = std::variant<CallbackHandler<Ts>...>;
};
}  // namespace detail

template <typename T>
using CallbackHandlerType = typename detail::CallbackHandlerType<T>::type;

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
    std::lock_guard guard(g_mutex);

    server_ = node.create_service<std_srvs::srv::Empty>(
      "/autoware_test_utils/topic_snapshot_saver",
      std::bind(
        &TopicSnapShotSaver::on_service, this, std::placeholders::_1, std::placeholders::_2));

    // get map version
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
      field_2_topic_type_[name] = type_index;

      /*
      if (0 == type_index) {
        CallbackHandler<typename std::variant_alternative_t<0, MessageType>> handler(
          std::ref(mutex_), std::ref(message_buffer_.at(0)));
        handlers_.emplace_back(handler);
        auto & handler_ref =
          std::get<CallbackHandler<typename std::variant_alternative_t<0, MessageType>>>(
            handlers_.back());
        subscribers_[0] = create_subscriber<0>(
          topic, node,
          std::bind(
            &CallbackHandler<std::variant_alternative_t<0, MessageType>>::on_callback, &handler_ref,
            std::placeholders::_1));
      }
      */

      /**
       * NOTE: for a specific topic-type, only one topic-name is allowed
       *
       * `message_buffer_` holds buffer for each topic-type for the lifetime duration of this class
       *
       * `subscribers_` holds a subscriber for each topic-type for the lifetime duration of this
       * class
       *
       * `handlers_` holds a handler for each topic-type for the lifetime duration of this
       * class
       *
       */

#define REGISTER_CALLBACK(arg)                                                                     \
  if (arg == type_index) {                                                                         \
    CallbackHandler<typename std::variant_alternative_t<arg, MessageType>> handler(                \
      std::ref(message_buffer_.at(arg)));                                                          \
    handlers_.emplace_back(handler);                                                               \
    auto & handler_ref =                                                                           \
      std::get<CallbackHandler<typename std::variant_alternative_t<arg, MessageType>>>(            \
        handlers_.back());                                                                         \
    subscribers_[arg] = create_subscriber<arg>(                                                    \
      topic, node,                                                                                 \
      std::bind(                                                                                   \
        &CallbackHandler<std::variant_alternative_t<arg, MessageType>>::on_callback, &handler_ref, \
        std::placeholders::_1));                                                                   \
  }

      REGISTER_CALLBACK(0);
      REGISTER_CALLBACK(1);
      REGISTER_CALLBACK(2);
      REGISTER_CALLBACK(3);
      REGISTER_CALLBACK(4);
      REGISTER_CALLBACK(5);
    }
  }

private:
  const std::string map_path_;
  const std::string config_path_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_;

  std::unordered_map<size_t, rclcpp::SubscriptionBase::SharedPtr> subscribers_;
  std::vector<CallbackHandlerType<MessageType>> handlers_;
  std::array<MessageType, std::variant_size_v<MessageType>> message_buffer_;
  std::unordered_map<std::string, size_t> field_2_topic_type_;
  void on_service(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    std::lock_guard guard(g_mutex);

    YAML::Node yaml;

    yaml["format_version"] = 1;

    for (const auto & [field, type_index] : field_2_topic_type_) {
      if (0 == type_index) {
        const auto & msg = std::get<typename std::variant_alternative_t<0, MessageType>>(
          message_buffer_.at(type_index));
        yaml[field] = to_yaml(msg);  // NOTE: ADL works fine!
      }
    }

    const std::string desc = std::string(R"(#
# AUTO GENERATED by autoware_test_utils::topic_snapshot_saver
# format1:
#
# format_version: <format-major-version, int>
# fields(this is array)
#   - name: <field-name-for-your-yaml-of-this-topic, str>
#     type: either {Odometry | AccelWithCovarianceStamped | PredictedObjects | OperationModeState | LaneletRoute | TrafficLightGroupArray}
#     topic: <topic-name, str>
#
)");

    std::ofstream ofs("topic_snapshot.yaml");
    ofs << desc;
    ofs << yaml;

    std::cout << "saved planner_data" << std::endl;
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
      RCLCPP_ERROR(get_logger(), "map_path_uri and/or config_path_uri are not provided");
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

  rclcpp::executors::SingleThreadedExecutor exec;

  auto node = std::make_shared<TopicSnapShotSaverFrontEnd>();

  exec.add_node(node);

  exec.spin();

  rclcpp::shutdown();
}
