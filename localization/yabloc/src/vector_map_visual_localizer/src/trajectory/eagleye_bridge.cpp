#include "common/util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace trajectory
{

class EagleyeBridge : public rclcpp::Node
{
public:
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using NavPVT = ublox_msgs::msg::NavPVT;

  EagleyeBridge() : Node("eagleye_bridge")
  {
    using std::placeholders::_1;

    // Subscriber
    auto fix_cb = std::bind(&EagleyeBridge::fixCallback, this, _1);
    sub_fix_ = create_subscription<NavSatFix>("/in/fix", 10, fix_cb);

    // Publisher
    pub_navpvt_ = this->create_publisher<NavPVT>("/out/navpvt", 10);
  }

private:
  rclcpp::Subscription<NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Publisher<NavPVT>::SharedPtr pub_navpvt_;

  void fixCallback(const NavSatFix & msg)
  {
    NavPVT navpvt = util::stamp2UbloxTime(msg.header.stamp);
    navpvt.flags = 0;
    navpvt.lat = msg.latitude * 1e7f;
    navpvt.lon = msg.longitude * 1e7f;
    navpvt.height = msg.altitude * 1e3f;
    pub_navpvt_->publish(navpvt);
  }
};

}  // namespace trajectory
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trajectory::EagleyeBridge>());
  rclcpp::shutdown();
  return 0;
}
