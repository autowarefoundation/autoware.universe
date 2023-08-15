#ifndef AUTOWARE_BAG_RECORDER_HPP
#define AUTOWARE_BAG_RECORDER_HPP


#include <iostream>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include <string>
#include <vector>
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"

#include <sys/statfs.h>

#include "rosbag2_cpp/writer.hpp"
#include <rosbag2_storage/rosbag2_storage/storage_options.hpp>


namespace autoware_bag_recorder {

struct TopicInfo{
  std::string topic_name;
  std::string topic_type;
};

struct ModuleSection{
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
  void run();

  std::string get_timestamp();
  rclcpp::QoS get_qos_profile_of_topic(const std::string & topic_name);
  void search_topic(ModuleSection & section);
  void create_bag_file(std::unique_ptr<rosbag2_cpp::Writer> & writer, const std::string &bag_path);
  void add_topics_to_writer(
      std::unique_ptr<rosbag2_cpp::Writer> & writer_, std::string topic_name, std::string topic_type);
  void generic_subscription_callback(
      const std::shared_ptr<rclcpp::SerializedMessage const> msg,
      const std::string& topic_name, rcutils_allocator_t allocator,
      autoware_bag_recorder::ModuleSection & section);
  void section_factory(std::vector<std::string> topics, std::string path);
  int get_root_disk_space();

  int maximum_time_;
  std::string bag_path_;
  bool record_planning_topics_;
  bool record_sensing_topics_;

  int remaining_topic_num_;
  int disk_space_threshold_;

  std::vector<std::string> planning_topics_;
  std::vector<std::string> sensing_topics_;

  std::vector<std::vector<std::string>> topics_of_each_module_;
  std::vector<ModuleSection> module_sections_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<rclcpp::SerializedMessage> serialized_msg_ptr_;

  std::mutex writer_mutex_;
};

}
#endif // AUTOWARE_BAG_RECORDER_HPP
