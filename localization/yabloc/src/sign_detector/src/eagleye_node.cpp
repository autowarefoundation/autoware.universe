#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
class EagleyeSubscriber : public rclcpp::Node
{
public:
  EagleyeSubscriber(const std::string& node_name) : Node(node_name)
  {
    std::string fix_topic = declare_parameter("fix_topic", "/eagleye/fix");
    std::string path_topic = declare_parameter("path_topic", "/eagleye/path");

    sub_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(fix_topic, 10, std::bind(&EagleyeSubscriber::fixCallback, this, std::placeholders::_1));
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  nav_msgs::msg::Path path_;

  void fixCallback(const sensor_msgs::msg::NavSatFix& msg)
  {
    using namespace GeographicLib;
    double x, y;
    int zone;
    bool northp;
    std::string mgrs;
    UTMUPS::Forward(msg.latitude, msg.longitude, zone, northp, x, y);
    MGRS::Forward(zone, northp, x, y, msg.latitude, 6, mgrs);

    double local_x = std::stoi(mgrs.substr(5, 6)) * 0.1;
    double local_y = std::stoi(mgrs.substr(11, 6)) * 0.1;

    RCLCPP_INFO_STREAM(this->get_logger(), mgrs.substr(0, 5) << " " << local_x << " " << local_y << " (" << msg.latitude << ", " << msg.longitude << ")");

    path_.header.frame_id = "map";
    path_.header.stamp = this->get_clock()->now();
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.orientation.w = 1;
    ps.pose.position.x = local_x;
    ps.pose.position.y = local_y;
    ps.pose.position.z = 0;
    path_.poses.push_back(ps);
    pub_path_->publish(path_);

    std::cout << "hoge" << std::endl;

    publishTf(ps);
  }

  void publishTf(const geometry_msgs::msg::PoseStamped& ps)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "eagleye";
    t.transform.translation.x = ps.pose.position.x;
    t.transform.translation.y = ps.pose.position.y;
    t.transform.translation.z = ps.pose.position.z;
    t.transform.rotation.w = 1;

    tf_broadcaster_->sendTransform(t);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EagleyeSubscriber>("eagleye_node"));
  rclcpp::shutdown();
  return 0;
}
