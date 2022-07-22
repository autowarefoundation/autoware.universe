#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "validation/ape.hpp"

#include <filesystem>
#include <sstream>
namespace validation
{
AbsolutePoseError::AbsolutePoseError() : Node("ape_node")
{
  using std::placeholders::_1;
  auto cb_pose = std::bind(&AbsolutePoseError::poseCallback, this, _1);
  sub_pose_cov_stamped_ = create_subscription<PoseCovStamped>("/pose_with_covariance", 1, cb_pose);
  pub_string_ = create_publisher<String>("/ape_diag", 10);

  const std::string reference_bags_path = declare_parameter<std::string>("reference_bags_path", "");
  RCLCPP_INFO_STREAM(get_logger(), "reference bag path: " << reference_bags_path);

  namespace fs = std::filesystem;
  for (const fs::directory_entry & x : fs::directory_iterator(reference_bags_path)) {
    loadRosbag(x.path());
  }
  RCLCPP_INFO_STREAM(get_logger(), "successed to read " << references_.size());
}

void AbsolutePoseError::loadRosbag(const std::string & bag_file)
{
  RCLCPP_INFO_STREAM(get_logger(), "opeing " << bag_file.size());

  Reference reference;
  rosbag2_cpp::Reader reader;
  reader.open(bag_file);
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    if (bag_message->topic_name == "/localization/kinematic_state") {
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      Odometry odom_msg;
      rclcpp::Serialization<Odometry> serialization;
      serialization.deserialize_message(&serialized_msg, &odom_msg);

      PoseStamped pose_msg;
      pose_msg.header = odom_msg.header;
      pose_msg.pose = odom_msg.pose.pose;
      reference.poses_.push_back(pose_msg);
    }
  }
  if (reference.poses_.empty()) return;

  reference.start_stamp_ = reference.poses_.front().header.stamp;
  reference.end_stamp_ = reference.poses_.back().header.stamp;
  references_[bag_file] = reference;
}

void AbsolutePoseError::poseCallback(const PoseCovStamped & pose_cov)
{
  std::stringstream ss;
  // TODO:
  ss << "APE: " << pose_cov.pose.pose.position.x;
  String msg;
  msg.data = ss.str();
  pub_string_->publish(msg);
}

}  // namespace validation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<validation::AbsolutePoseError>());
  rclcpp::shutdown();
  return 0;
}