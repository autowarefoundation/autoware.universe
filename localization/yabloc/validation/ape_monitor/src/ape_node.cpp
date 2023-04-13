// Copyright 2023 TIER IV, Inc.
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

#include "ape/ape.hpp"

#include <pcdless_common/pose_conversions.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include <filesystem>
#include <sstream>

namespace pcdless::ape_monitor
{
AbsolutePoseError::AbsolutePoseError() : Node("ape_node")
{
  using std::placeholders::_1;
  namespace fs = std::filesystem;

  auto cb_pose = std::bind(&AbsolutePoseError::on_pose, this, _1);
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
    load_reference_rosbag(x.path());
  }
  RCLCPP_INFO_STREAM(get_logger(), "successed to read " << references_.size());
}

void AbsolutePoseError::load_reference_rosbag(const std::string & bag_file)
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

Eigen::Vector2f AbsolutePoseError::compute_ape(
  const Reference & ref, const PoseCovStamped & pose_cov) const
{
  PoseStamped stamped;
  stamped.header = pose_cov.header;

  auto itr = std::upper_bound(
    ref.poses_.begin(), ref.poses_.end(), stamped,
    [](const PoseStamped & a, const PoseStamped & b) -> bool {
      return rclcpp::Time(a.header.stamp) < rclcpp::Time(b.header.stamp);
    });

  Eigen::Affine3f ref_affine = common::pose_to_affine(itr->pose);
  Eigen::Affine3f est_affine = common::pose_to_affine(pose_cov.pose.pose);

  Eigen::Affine3f diff_affine = ref_affine.inverse() * est_affine;
  Eigen::Vector3f ape = diff_affine.translation();
  return ape.cwiseAbs().topRows(2);
}

void AbsolutePoseError::on_pose(const PoseCovStamped & pose_cov)
{
  const rclcpp::Time stamp = pose_cov.header.stamp;
  std::optional<Reference> corresponding_reference{std::nullopt};

  for (const Reference & ref : references_) {
    if ((stamp > ref.start_stamp_) & (stamp < ref.end_stamp_)) corresponding_reference = ref;
  }

  std::stringstream ss;
  ss << "--- APE Status ---" << std::endl;
  if (corresponding_reference.has_value()) {
    Eigen::Vector2f ape = compute_ape(corresponding_reference.value(), pose_cov);
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

}  // namespace pcdless::ape_monitor

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::ape_monitor::AbsolutePoseError>());
  rclcpp::shutdown();
  return 0;
}