#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class Pose2Path : public rclcpp::Node
{
public:
  Pose2Path() : Node("pose_to_path")
  {
    std::string path_topic = "/eagleye/path";
    std::string pose_topic = "/eagleye/pose";
    this->declare_parameter<std::string>("path_topic", path_topic);
    this->declare_parameter<std::string>("pose_topic", pose_topic);
    this->get_parameter("path_topic", path_topic);
    this->get_parameter("pose_topic", pose_topic);

    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&Pose2Path::poseCallback, this, std::placeholders::_1));
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped& msg)
  {
    path_.header = msg.header;
    path_.poses.push_back(msg);
    pub_path_->publish(path_);
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Pose2Path>());
  rclcpp::shutdown();
  return 0;
}
