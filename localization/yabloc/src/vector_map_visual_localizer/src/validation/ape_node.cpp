#include "validation/ape.hpp"

#include <sstream>

namespace validation
{
AbsolutePoseError::AbsolutePoseError()
: Node("ape_node"), reference_bags_path_(declare_parameter<std::string>("reference_bags_path", ""))
{
  using std::placeholders::_1;
  auto cb_pose = std::bind(&AbsolutePoseError::poseCallback, this, _1);
  sub_pose_cov_stamped_ = create_subscription<PoseCovStamped>("/pose_with_covariance", 10, cb_pose);
  pub_string_ = create_publisher<String>("/ape_diag", 10);

  RCLCPP_INFO_STREAM(get_logger(), "reference bag path: " << reference_bags_path_);
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