#include "twist_estimator/twist_visualizer.hpp"

namespace pcdless::twist_visualizer
{
TwistVisualizer::TwistVisualizer() : Node("twist_visualizer"), odoms_{100}
{
  using std::placeholders::_1;
  using namespace std::literals::chrono_literals;

  auto cb_twist = std::bind(&TwistVisualizer::on_twist_stamped, this, _1);
  sub_twist_stamped_ = create_subscription<TwistStamped>("/vehicle/status/twist", 10, cb_twist);

  pub_path_ = create_publisher<Path>("odom_path", 10);
}

void TwistVisualizer::on_twist_stamped(const TwistStamped & msg)
{
  if (!last_twist_stamp_) {
    last_twist_stamp_ = msg.header.stamp;
    return;
  }

  const float dt = (rclcpp::Time(msg.header.stamp) - *(last_twist_stamp_)).seconds();

  Eigen::Vector3f w, v;
  auto linear = msg.twist.linear;
  auto angular = msg.twist.angular;
  v << linear.x, linear.y, linear.z;
  w << angular.x, angular.y, angular.z;

  last_odom_ *= Sophus::SE3f(Sophus::SO3f::exp(w * dt), v * dt);
  last_twist_stamp_ = msg.header.stamp;

  if (last_odom_.translation().norm() > 0.5) {
    odoms_.push_front(last_odom_);
    last_odom_ = Sophus::SE3f();
    publish_path(msg.header.stamp);
  }
}

void TwistVisualizer::publish_path(const rclcpp::Time & stamp)
{
  Path msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "base_link";

  Sophus::SE3f absolute_pose;
  for (const Sophus::SE3f & relative_pose : odoms_) {
    PoseStamped pose_stamped;
    absolute_pose *= relative_pose.inverse();
    pose_stamped.pose = se3f_to_pose_msg(absolute_pose);
    msg.poses.push_back(pose_stamped);
  }
  pub_path_->publish(msg);
}

TwistVisualizer::Pose TwistVisualizer::se3f_to_pose_msg(const Sophus::SE3f & pose)
{
  Pose msg;
  const Eigen::Quaternionf q = pose.unit_quaternion();
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  const Eigen::Vector3f t = pose.translation();
  msg.position.x = t.x();
  msg.position.y = t.y();
  msg.position.z = t.z();
  return msg;
}

}  // namespace pcdless::twist_visualizer