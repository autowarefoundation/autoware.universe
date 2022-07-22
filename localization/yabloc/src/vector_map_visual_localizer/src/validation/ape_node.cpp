#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "validation/ape.hpp"

#include <sstream>

namespace validation
{
AbsolutePoseError::AbsolutePoseError()
: Node("ape_node"), reference_bags_path_(declare_parameter<std::string>("reference_bags_path", ""))
{
  using std::placeholders::_1;
  auto cb_pose = std::bind(&AbsolutePoseError::poseCallback, this, _1);
  sub_pose_cov_stamped_ = create_subscription<PoseCovStamped>("/pose_with_covariance", 1, cb_pose);
  pub_string_ = create_publisher<String>("/ape_diag", 10);

  RCLCPP_INFO_STREAM(get_logger(), "reference bag path: " << reference_bags_path_);

  {
    std::cout << "try to read rosbag2" << std::endl;
    rosbag2_cpp::Reader reader;
    reader.open(reference_bags_path_);
    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      if (bag_message->topic_name == "/localization/kinematic_state") {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        Odometry msg;
        rclcpp::Serialization<Odometry> serialization;
        serialization.deserialize_message(&serialized_msg, &msg);
      }
    }
    std::cout << "successed to read rosbag2" << std::endl;
  }
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