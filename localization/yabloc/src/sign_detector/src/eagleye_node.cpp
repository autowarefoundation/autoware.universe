#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class EagleyeSubscriber : public rclcpp::Node
{
public:
  EagleyeSubscriber(const std::string& fix_topic, const std::string& twist_topic) : Node("eagleye_subscriber")
  {
    sub_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(fix_topic, 10, std::bind(&EagleyeSubscriber::fixCallback, this, std::placeholders::_1));
    sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(twist_topic, 10, std::bind(&EagleyeSubscriber::twistCallback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;

  void fixCallback(const sensor_msgs::msg::NavSatFix& msg) const
  {
    using namespace GeographicLib;
    double x, y;
    int zone;
    bool northp;
    std::string mgrs;
    UTMUPS::Forward(msg.latitude, msg.longitude, zone, northp, x, y);
    MGRS::Forward(zone, northp, x, y, msg.latitude, 5, mgrs);
    std::cout << msg.latitude << " " << msg.longitude << " " << msg.altitude << std::endl;
    std::cout << mgrs << " " << x << " " << y << std::endl;
  }

  void twistCallback(const geometry_msgs::msg::TwistStamped&) const
  {
    // std::cout << msg.header.frame_id << std::endl;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const std::string fix_topic = "/eagleye/fix";
  const std::string twist_topic = "/eagleye/twist";
  rclcpp::spin(std::make_shared<EagleyeSubscriber>(fix_topic, twist_topic));
  rclcpp::shutdown();
  return 0;
}
