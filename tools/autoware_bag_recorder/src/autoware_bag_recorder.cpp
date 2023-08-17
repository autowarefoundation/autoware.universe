// Copyright 2023 TIER IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "autoware_bag_recorder.hpp"

namespace autoware_bag_recorder
{

AutowareBagRecorderNode::AutowareBagRecorderNode(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options), node_(this)
{
  // common params declarations
  bag_path_ = declare_parameter<std::string>("common.path");
  disk_space_threshold_ = declare_parameter<int>("common.check_disk_space_threshold");
  maximum_record_time_ = declare_parameter<int>("common.maximum_record_time");
  bag_time_ = declare_parameter<int>("common.bag_time");
  number_of_maximum_bags_ = declare_parameter<int>("common.number_of_maximum_bags");
  record_all_topic_in_a_bag_ = declare_parameter<bool>("common.record_all_topic_in_a_bag");
  disk_space_action_mode_ = declare_parameter<std::string>("common.disk_space_threshold_action");
  // api
  record_api_topics_ = declare_parameter<bool>("api_modules.record_api");
  if (record_api_topics_ || record_all_topic_in_a_bag_) {
    api_topics_ = declare_parameter<std::vector<std::string>>("api_modules.api_topics");
    section_factory(api_topics_, bag_path_ + "api");
  }

  // autoware
  record_autoware_topics_ = declare_parameter<bool>("autoware_modules.record_autoware");
  if (record_autoware_topics_ || record_all_topic_in_a_bag_) {
    autoware_topics_ =
      declare_parameter<std::vector<std::string>>("autoware_modules.autoware_topics");
    section_factory(autoware_topics_, bag_path_ + "autoware");
  }

  // control
  record_control_topics_ = declare_parameter<bool>("control_modules.record_control");
  if (record_control_topics_ || record_all_topic_in_a_bag_) {
    control_topics_ = declare_parameter<std::vector<std::string>>("control_modules.control_topics");
    section_factory(control_topics_, bag_path_ + "control");
  }

  // external
  record_external_topics_ = declare_parameter<bool>("external_modules.record_external");
  if (record_external_topics_ || record_all_topic_in_a_bag_) {
    external_topics_ =
      declare_parameter<std::vector<std::string>>("external_modules.external_topics");
    section_factory(external_topics_, bag_path_ + "external");
  }

  // localization
  record_localization_topics_ = declare_parameter<bool>("localization_modules.record_localization");
  if (record_localization_topics_ || record_all_topic_in_a_bag_) {
    localization_topics_ =
      declare_parameter<std::vector<std::string>>("localization_modules.localization_topics");
    section_factory(localization_topics_, bag_path_ + "localization");
  }

  // map
  record_map_topics_ = declare_parameter<bool>("map_modules.record_map");
  if (record_map_topics_ || record_all_topic_in_a_bag_) {
    map_topics_ = declare_parameter<std::vector<std::string>>("map_modules.map_topics");
    section_factory(map_topics_, bag_path_ + "map");
  }

  // perception
  record_perception_topics_ = declare_parameter<bool>("perception_modules.record_perception");
  if (record_perception_topics_ || record_all_topic_in_a_bag_) {
    perception_topics_ =
      declare_parameter<std::vector<std::string>>("perception_modules.perception_topics");
    section_factory(perception_topics_, bag_path_ + "perception");
  }

  // planning
  record_planning_topics_ = declare_parameter<bool>("planning_modules.record_planning");
  if (record_planning_topics_ || record_all_topic_in_a_bag_) {
    planning_topics_ =
      declare_parameter<std::vector<std::string>>("planning_modules.planning_topics");
    section_factory(planning_topics_, bag_path_ + "planning");
  }

  // sensing
  record_sensing_topics_ = declare_parameter<bool>("sensing_modules.record_sensing");
  if (record_sensing_topics_ || record_all_topic_in_a_bag_) {
    sensing_topics_ = declare_parameter<std::vector<std::string>>("sensing_modules.sensing_topics");
    section_factory(sensing_topics_, bag_path_ + "sensing");
  }

  // system
  record_system_topics_ = declare_parameter<bool>("system_modules.record_system");
  if (record_system_topics_ || record_all_topic_in_a_bag_) {
    system_topics_ = declare_parameter<std::vector<std::string>>("system_modules.system_topics");
    section_factory(system_topics_, bag_path_ + "system");
  }

  // vehicle
  record_vehicle_topics_ = declare_parameter<bool>("vehicle_modules.record_vehicle");
  if (record_vehicle_topics_ || record_all_topic_in_a_bag_) {
    vehicle_topics_ = declare_parameter<std::vector<std::string>>("vehicle_modules.vehicle_topics");
    section_factory(vehicle_topics_, bag_path_ + "vehicle");
  }

  if (record_all_topic_in_a_bag_) {
    ModuleSection all_topics;
    std::vector<std::string> all_topic_names;
    for (const auto & section : module_sections_) {
      for (const auto & topic : section.topic_names) {
        all_topic_names.push_back(topic);
      }
    }
    module_sections_ = std::vector<ModuleSection>();
    section_factory(all_topic_names, bag_path_ + "all");
  }

  for (auto & section : module_sections_) {
    remove_remainder_bags_in_folder(section);
  }

  remaining_topic_num_ = 0;

  run();
}

std::string AutowareBagRecorderNode::get_timestamp()
{
  char timestamp_str[100];
  std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::strftime(
    timestamp_str, sizeof(timestamp_str), "%Y-%m-%d-%H-%M-%S", std::localtime(&now_time_t));
  return timestamp_str;
}

void AutowareBagRecorderNode::create_bag_file(
  std::unique_ptr<rosbag2_cpp::Writer> & writer, const std::string & bag_path)
{
  writer = std::make_unique<rosbag2_cpp::Writer>();

  rosbag2_storage::StorageOptions storage_options_new;
  storage_options_new.uri = bag_path;
  storage_options_new.storage_id = "sqlite3";
  writer->open(storage_options_new);
}

void AutowareBagRecorderNode::section_factory(std::vector<std::string> topics, std::string path)
{
  ModuleSection section;
  for (size_t i = 0; i < topics.size(); ++i) {
    TopicInfo topic_info = {topics[i], ""};
    section.topic_info.push_back(topic_info);
    section.topic_names.push_back(topics[i]);
  }

  if (!section.topic_names.empty()) {
    section.folder_path = path;
    section.bag_writer = std::make_unique<rosbag2_cpp::Writer>();

    if (!std::filesystem::exists(path)) {
      std::filesystem::create_directories(path);
    }
    module_sections_.emplace_back(std::move(section));
  }
}

void AutowareBagRecorderNode::add_topics_to_writer(
  std::unique_ptr<rosbag2_cpp::Writer> & writer_, std::string topic_name, std::string topic_type)
{
  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = topic_name;

  topic_metadata.type = topic_type;
  topic_metadata.serialization_format = "cdr";

  writer_->create_topic(topic_metadata);
}

rclcpp::QoS AutowareBagRecorderNode::get_qos_profile_of_topic(const std::string & topic_name)
{
  auto publisher_info = this->get_publishers_info_by_topic(topic_name);
  auto subscriber_info = this->get_subscriptions_info_by_topic(topic_name);
  if (publisher_info.size() > 0) {
    return publisher_info[0].qos_profile();
  } else {
    return subscriber_info[0].qos_profile();
  }
}

void AutowareBagRecorderNode::generic_subscription_callback(
  const std::shared_ptr<rclcpp::SerializedMessage const> msg, const std::string & topic_name,
  autoware_bag_recorder::ModuleSection & section)
{
  auto serialized_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  serialized_bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
  serialized_bag_msg->topic_name = topic_name;
  serialized_bag_msg->time_stamp = node_->now().nanoseconds();
  serialized_bag_msg->serialized_data->buffer = msg->get_rcl_serialized_message().buffer;
  serialized_bag_msg->serialized_data->buffer_length =
    msg->get_rcl_serialized_message().buffer_length;
  serialized_bag_msg->serialized_data->buffer_capacity =
    msg->get_rcl_serialized_message().buffer_capacity;
  serialized_bag_msg->serialized_data->allocator = msg->get_rcl_serialized_message().allocator;

  std::lock_guard<std::mutex> lock(writer_mutex_);
  section.bag_writer->write(serialized_bag_msg);
}

void AutowareBagRecorderNode::search_topic(autoware_bag_recorder::ModuleSection & section)
{
  std::string topic_name = section.topic_names.front();
  auto topic_info_map = node_->get_topic_names_and_types();
  std::string topic_type;
  for (const auto & topic_info : topic_info_map) {
    if (topic_info.first == topic_name) {
      topic_type = topic_info.second[0];
      break;
    }
  }

  if (!topic_type.empty()) {
    add_topics_to_writer(section.bag_writer, topic_name, topic_type);

    auto topics_interface = node_->get_node_topics_interface();
    RCLCPP_INFO(
      get_logger(), "Subscribed topic %s of type %s", topic_name.c_str(), topic_type.c_str());
    auto subscription = rclcpp::create_generic_subscription(
      topics_interface, topic_name, topic_type, get_qos_profile_of_topic(topic_name),
      [this, topic_name, &section](const std::shared_ptr<rclcpp::SerializedMessage const> msg) {
        generic_subscription_callback(msg, topic_name, section);
      });

    // write_bag_message(section, topic_name);
    subscriptions_.push_back(subscription);
    remaining_topic_num_ = remaining_topic_num_ - 1;

    for (auto & topic_info : section.topic_info) {
      if (topic_info.topic_name == topic_name) {
        topic_info.topic_type = topic_type;
      }
    }
    section.topic_names.erase(section.topic_names.begin());
  } else {
    if (!section.topic_names.empty()) {
      std::rotate(
        section.topic_names.rbegin(), section.topic_names.rbegin() + 1, section.topic_names.rend());
    }
  }
}

int AutowareBagRecorderNode::get_root_disk_space()
{
  std::filesystem::space_info root = std::filesystem::space("/");

  return (root.available / pow(1024.0, 3.0));  // Convert to GB
}

void AutowareBagRecorderNode::check_files_in_folder(
  autoware_bag_recorder::ModuleSection & section,std::vector<std::string> & directories)
{
  for (const auto & path : std::filesystem::recursive_directory_iterator(section.folder_path)) {
    if (path.is_directory()) {
      directories.push_back(path.path().string());
    }
  }

  std::sort(
    directories.begin(), directories.end(),
    [](const std::string & a, const std::string & b) -> bool { return a < b; });
}

void AutowareBagRecorderNode::remove_remainder_bags_in_folder(
  autoware_bag_recorder::ModuleSection & section)
{
  std::vector<std::string> directories;
  check_files_in_folder(section, directories);

  while (directories.size() >= static_cast<std::vector<int>::size_type>(number_of_maximum_bags_)) {
    std::filesystem::remove_all(directories[0]);
    directories.erase(directories.begin());
  }
}

void AutowareBagRecorderNode::free_disk_space_for_continue(
  autoware_bag_recorder::ModuleSection & section)
{
  std::vector<std::string> directories;
  check_files_in_folder(section, directories);

  while (get_root_disk_space() < disk_space_threshold_) {
    std::filesystem::remove_all(directories[0]);
    directories.erase(directories.begin());
  }
}

void AutowareBagRecorderNode::run()
{
  for (auto & section : module_sections_) {
    create_bag_file(section.bag_writer, section.folder_path + "/rosbag2_" + get_timestamp());
    remaining_topic_num_ = remaining_topic_num_ + section.topic_names.size();
  }

  std::thread([this]() {
    rclcpp::Rate rate(100);
    // if all topics are not subscribed, then continue checking
    while (remaining_topic_num_ > 0) {
      for (auto & section : module_sections_) {
        search_topic(section);
      }
      rate.sleep();
    }
  }).detach();

  std::thread([this]() {
    rclcpp::Rate rate(10);

    auto start_record_time = std::chrono::system_clock::now();
    auto start_bag_time = std::chrono::system_clock::now();

    while (rclcpp::ok()) {
      // check available disk space, if current disk space is smaller than threshold,
      // then shutdown node
      if (get_root_disk_space() < disk_space_threshold_) {
        RCLCPP_WARN(
          this->get_logger(), "Available Disk Space is: %d under the threshold.",
          get_root_disk_space());
        if (disk_space_action_mode_ == "remove")
        {
          for (auto & section : module_sections_)
          {
            free_disk_space_for_continue(section);
          }
        }
        rclcpp::shutdown();
      }

      // check record time, if record time is exceeded then shutdown node
      if (
        (std::chrono::system_clock::now() - start_record_time) >
        std::chrono::seconds(maximum_record_time_)) {
        RCLCPP_WARN(this->get_logger(), "The maximum record time is reached.");
        rclcpp::shutdown();
      }

      if ((std::chrono::system_clock::now() - start_bag_time) >= std::chrono::seconds(bag_time_)) {
        start_bag_time = std::chrono::system_clock::now();

        for (auto & section : module_sections_) {
          remove_remainder_bags_in_folder(section);
          std::lock_guard<std::mutex> lock(writer_mutex_);
          create_bag_file(section.bag_writer, section.folder_path + "/rosbag2_" + get_timestamp());
          for (auto & topic_info : section.topic_info) {
            add_topics_to_writer(section.bag_writer, topic_info.topic_name, topic_info.topic_type);
          }
        }
      }
      rate.sleep();
    }
  }).detach();
}

}  // namespace autoware_bag_recorder
