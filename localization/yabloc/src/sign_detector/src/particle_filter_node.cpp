#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ParticleFilter : public rclcpp::Node
{
public:
  ParticleFilter() : Node("particle_filter"), frame_id_("map"), child_frame_id_("base_link")
  {
    std::string pose_topic = "/eagleye/pose";
    this->declare_parameter<std::string>("pose_topic", pose_topic);
    this->get_parameter("pose_topic", pose_topic);

    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&ParticleFilter::poseCallback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string frame_id_, child_frame_id_;

  void publishTf(const geometry_msgs::msg::PoseStamped& pose)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = frame_id_;
    t.child_frame_id = child_frame_id_;
    t.transform.translation.x = pose.pose.position.x;
    t.transform.translation.y = pose.pose.position.y;
    t.transform.translation.z = pose.pose.position.z;
    t.transform.rotation.w = pose.pose.orientation.w;
    t.transform.rotation.x = pose.pose.orientation.x;
    t.transform.rotation.y = pose.pose.orientation.y;
    t.transform.rotation.z = pose.pose.orientation.z;

    tf_broadcaster_->sendTransform(t);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped& msg)
  {
    publishTf(msg);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParticleFilter>());
  rclcpp::shutdown();
  return 0;
}
