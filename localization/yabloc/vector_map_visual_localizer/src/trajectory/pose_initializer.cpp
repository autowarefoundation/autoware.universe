#include "common/util.hpp"

#include <rclcpp/rclcpp.hpp>
#include <vml_common/fix2mgrs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

class PoseInitializer : public rclcpp::Node
{
public:
  using NavPVT = ublox_msgs::msg::NavPVT;
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  PoseInitializer() : Node("pose_initializer")
  {
    using std::placeholders::_1;

    // Subscriber
    auto navpvt_cb = std::bind(&PoseInitializer::navpvtCallback, this, _1);
    sub_navpvt_ = create_subscription<NavPVT>("/ublox_topic", 10, navpvt_cb);

    // Publisher
    pub_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
  }

private:
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr sub_navpvt_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_stamped_;

  void navpvtCallback(const NavPVT & msg)
  {
    const int FIX_FLAG = NavPVT::CARRIER_PHASE_FIXED;
    const int FLOAT_FLAG = NavPVT::CARRIER_PHASE_FLOAT;
    if (!(msg.flags & FIX_FLAG) & !(msg.flags & FLOAT_FLAG)) return;

    // Compute error and jacobian
    Eigen::Vector2f vel_xy = extractEnuVel(msg).topRows(2);
    float theta = std::atan2(vel_xy.y(), vel_xy.x());

    Eigen::Vector3f position = ublox2Position(msg);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = util::ubloxTime2Stamp(msg);
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = 0;
    pose.pose.orientation.w = std::cos(theta / 2.f);
    pose.pose.orientation.z = std::sin(theta / 2.f);
    pub_pose_stamped_->publish(pose);
  }

  Eigen::Vector3f extractEnuVel(const NavPVT & msg) const
  {
    Eigen::Vector3f enu_vel;
    enu_vel << msg.vel_e * 1e-3, msg.vel_n * 1e-3, -msg.vel_d * 1e-3;
    return enu_vel;
  }

  Eigen::Vector3f ublox2Position(const NavPVT & msg)
  {
    NavSatFix fix;
    fix.latitude = msg.lat * 1e-7f;
    fix.longitude = msg.lon * 1e-7f;
    fix.altitude = msg.height * 1e-3f;
    return vml_common::fix2Mgrs(fix).cast<float>();
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseInitializer>());
  rclcpp::shutdown();
  return 0;
}
