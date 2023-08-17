#ifndef AUTOWARE_BAG_RECORDER_HPP_
#define AUTOWARE_BAG_RECORDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/writer.hpp"

#include <rosbag2_storage/rosbag2_storage/storage_options.hpp>

#include <sys/statfs.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace autoware_bag_recorder
{

struct TopicInfo
{
  std::string topic_name;
  std::string topic_type;
};

struct ModuleSection
{
  std::string folder_path;
  std::vector<TopicInfo> topic_info;
  std::unique_ptr<rosbag2_cpp::Writer> bag_writer;
  std::vector<std::string> topic_names;
};

class AutowareBagRecorderNode : public rclcpp::Node
{
public:
  AutowareBagRecorderNode(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  int maximum_record_time_;
  int bag_time_;
  std::string bag_path_;
  int disk_space_threshold_;
  int number_of_maximum_bags_;
  std::string disk_space_action_mode_;

  bool record_all_topic_in_a_bag_;

  bool record_api_topics_;
  bool record_autoware_topics_;
  bool record_control_topics_;
  bool record_external_topics_;
  bool record_localization_topics_;
  bool record_map_topics_;
  bool record_perception_topics_;
  bool record_planning_topics_;
  bool record_sensing_topics_;
  bool record_system_topics_;
  bool record_vehicle_topics_;

  int remaining_topic_num_;

  std::vector<std::string> api_topics_;
  std::vector<std::string> autoware_topics_;
  std::vector<std::string> control_topics_;
  std::vector<std::string> external_topics_;
  std::vector<std::string> localization_topics_;
  std::vector<std::string> map_topics_;
  std::vector<std::string> perception_topics_;
  std::vector<std::string> planning_topics_;
  std::vector<std::string> sensing_topics_;
  std::vector<std::string> system_topics_;
  std::vector<std::string> vehicle_topics_;

  std::vector<ModuleSection> module_sections_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<rclcpp::SerializedMessage> serialized_msg_ptr_;

  std::mutex writer_mutex_;

  std::string get_timestamp();
  rclcpp::QoS get_qos_profile_of_topic(const std::string & topic_name);
  void search_topic(ModuleSection & section);
  void create_bag_file(std::unique_ptr<rosbag2_cpp::Writer> & writer, const std::string & bag_path);
  void add_topics_to_writer(
    std::unique_ptr<rosbag2_cpp::Writer> & writer_, std::string topic_name, std::string topic_type);
  void generic_subscription_callback(
    const std::shared_ptr<rclcpp::SerializedMessage const> msg, const std::string & topic_name,
    autoware_bag_recorder::ModuleSection & section);
  void section_factory(std::vector<std::string> topics, std::string path);
  int get_root_disk_space();
  void remove_remainder_bags_in_folder(autoware_bag_recorder::ModuleSection & section);
  void free_disk_space_for_continue(autoware_bag_recorder::ModuleSection & section);
  void check_files_in_folder(
    autoware_bag_recorder::ModuleSection & section,std::vector<std::string> & directories);
  void run();
};

}  // namespace autoware_bag_recorder
#endif  // AUTOWARE_BAG_RECORDER_HPP_
