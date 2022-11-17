#include "ape/ape.hpp"

#include <pcdless_common/pose_conversions.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include <filesystem>
#include <sstream>

namespace vmvl_validation
{
AbsolutePoseError::AbsolutePoseError() : Node("ape_node")
{
  using std::placeholders::_1;
  namespace fs = std::filesystem;

  auto cb_pose = std::bind(&AbsolutePoseError::poseCallback, this, _1);
  sub_pose_cov_stamped_ = create_subscription<PoseCovStamped>("pose_with_cov", 1, cb_pose);
  pub_string_ = create_publisher<String>("ape_diag", 10);

  const std::string reference_bags_path = declare_parameter<std::string>("reference_bags_path", "");
  RCLCPP_INFO_STREAM(get_logger(), "reference bag path: " << reference_bags_path);

  if (!fs::exists(reference_bags_path)) {
    RCLCPP_INFO_STREAM(
      get_logger(), "ape_node is disabled due to not existance of reference paths");
    return;
  }

  for (const fs::directory_entry & x : fs::directory_iterator(reference_bags_path)) {
    loadReferenceRosbag(x.path());
  }
  RCLCPP_INFO_STREAM(get_logger(), "successed to read " << references_.size());
}

void AbsolutePoseError::loadReferenceRosbag(const std::string & bag_file)
{
  RCLCPP_INFO_STREAM(get_logger(), "opening " << bag_file.size());

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
  references_.push_back(std::move(reference));
}

Eigen::Vector2f AbsolutePoseError::computeApe(
  const Reference & ref, const PoseCovStamped & pose_cov) const
{
  PoseStamped stamped;
  stamped.header = pose_cov.header;

  auto itr = std::upper_bound(
    ref.poses_.begin(), ref.poses_.end(), stamped,
    [](const PoseStamped & a, const PoseStamped & b) -> bool {
      return rclcpp::Time(a.header.stamp) < rclcpp::Time(b.header.stamp);
    });

  Eigen::Affine3f ref_affine = vml_common::pose2Affine(itr->pose);
  Eigen::Affine3f est_affine = vml_common::pose2Affine(pose_cov.pose.pose);

  Eigen::Affine3f diff_affine = ref_affine.inverse() * est_affine;
  Eigen::Vector3f ape = diff_affine.translation();
  return ape.cwiseAbs().topRows(2);
}

void AbsolutePoseError::poseCallback(const PoseCovStamped & pose_cov)
{
  const rclcpp::Time stamp = pose_cov.header.stamp;
  std::optional<Reference> corresponding_reference{std::nullopt};

  for (const Reference & ref : references_) {
    if ((stamp > ref.start_stamp_) & (stamp < ref.end_stamp_)) corresponding_reference = ref;
  }

  std::stringstream ss;
  ss << "--- APE Status ---" << std::endl;
  if (corresponding_reference.has_value()) {
    Eigen::Vector2f ape = computeApe(corresponding_reference.value(), pose_cov);
    ss << std::fixed << std::setprecision(2);
    ss << "LONG: " << ape.x() << "\n";
    ss << "LATE: " << ape.y();
  } else {
    ss << "APE: NaN";
  }

  String msg;
  msg.data = ss.str();
  pub_string_->publish(msg);
}

}  // namespace vmvl_validation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vmvl_validation::AbsolutePoseError>());
  rclcpp::shutdown();
  return 0;
}