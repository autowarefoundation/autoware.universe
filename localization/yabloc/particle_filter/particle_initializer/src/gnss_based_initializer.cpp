#include <pcdless_common/fix2mgrs.hpp>
#include <pcdless_common/ublox_stamp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace pcdless::particle_initializer
{
class GnssBasedPoseInitializer : public rclcpp::Node
{
public:
  using NavPVT = ublox_msgs::msg::NavPVT;
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  GnssBasedPoseInitializer() : Node("gnss_based_initializer")
  {
    using std::placeholders::_1;

    // Subscriber
    auto navpvt_cb = std::bind(&GnssBasedPoseInitializer::on_navpvt, this, _1);
    sub_navpvt_ = create_subscription<NavPVT>("ublox_topic", 10, navpvt_cb);

    // Publisher
    pub_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
  }

private:
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr sub_navpvt_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_stamped_;

  void on_navpvt(const NavPVT & msg)
  {
    const int FIX_FLAG = NavPVT::CARRIER_PHASE_FIXED;
    const int FLOAT_FLAG = NavPVT::CARRIER_PHASE_FLOAT;
    if (!(msg.flags & FIX_FLAG) & !(msg.flags & FLOAT_FLAG)) return;

    Eigen::Vector2f vel_xy = extract_enu_vel(msg).topRows(2);
    if (vel_xy.norm() < 5) return;
    float theta = std::atan2(vel_xy.y(), vel_xy.x());

    Eigen::Vector3f position = ublox_to_position(msg);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = common::ublox_time_to_stamp(msg);
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = 0;
    pose.pose.orientation.w = std::cos(theta / 2.f);
    pose.pose.orientation.z = std::sin(theta / 2.f);
    pub_pose_stamped_->publish(pose);
  }

  Eigen::Vector3f extract_enu_vel(const NavPVT & msg) const
  {
    Eigen::Vector3f enu_vel;
    enu_vel << msg.vel_e * 1e-3, msg.vel_n * 1e-3, -msg.vel_d * 1e-3;
    return enu_vel;
  }

  Eigen::Vector3f ublox_to_position(const NavPVT & msg)
  {
    NavSatFix fix;
    fix.latitude = msg.lat * 1e-7f;
    fix.longitude = msg.lon * 1e-7f;
    fix.altitude = msg.height * 1e-3f;
    return common::fix_to_mgrs(fix).cast<float>();
  }
};
}  // namespace pcdless::particle_initializer

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::particle_initializer::GnssBasedPoseInitializer>());
  rclcpp::shutdown();
  return 0;
}
