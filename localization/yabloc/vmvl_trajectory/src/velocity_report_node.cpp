#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace vmvl_trajectory
{
class VelocityReportDecoder : public rclcpp::Node
{
public:
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Velocity = autoware_auto_vehicle_msgs::msg::VelocityReport;

  VelocityReportDecoder() : Node("vehicle_report")
  {
    using std::placeholders::_1;

    // Subscriber
    auto cb_velocity = std::bind(&VelocityReportDecoder::velocityCallback, this, _1);
    sub_velocity_ =
      create_subscription<Velocity>("/vehicle/status/velocity_status", 10, cb_velocity);

    // Publisher
    pub_twist_stamped_ = create_publisher<TwistStamped>("/vehicle/status/twist", 10);
  }

private:
  rclcpp::Subscription<Velocity>::SharedPtr sub_velocity_;
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_twist_stamped_;

  void velocityCallback(const Velocity & msg)
  {
    TwistStamped ts;
    ts.header = msg.header;
    ts.twist.linear.x = msg.longitudinal_velocity;
    ts.twist.linear.y = msg.lateral_velocity;
    ts.twist.angular.z = msg.heading_rate;
    pub_twist_stamped_->publish(ts);
  }
};
}  // namespace vmvl_trajectory

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vmvl_trajectory::VelocityReportDecoder>());
  rclcpp::shutdown();
  return 0;
}