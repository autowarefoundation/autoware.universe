#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ParticleFilter : public rclcpp::Node
{
public:
  ParticleFilter() : Node("particle_filter"), frame_id_("map"), child_frame_id_("base_link")
  {
    std::string pose_topic = "/eagleye/pose";
    this->declare_parameter<std::string>("pose_topic", pose_topic);
    this->get_parameter("pose_topic", pose_topic);

    sub_ll2_image_ = this->create_subscription<sensor_msgs::msg::Image>("/ll2_image", 10, std::bind(&ParticleFilter::ll2ImageCallback, this, std::placeholders::_1));
    sub_lsd_image_ = this->create_subscription<sensor_msgs::msg::Image>("/projected_image", 10, std::bind(&ParticleFilter::lsdImageCallback, this, std::placeholders::_1));

    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&ParticleFilter::poseCallback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pub_match_image_ = this->create_publisher<sensor_msgs::msg::Image>("/match_image", 10);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ll2_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_lsd_image_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string frame_id_, child_frame_id_;

  std::optional<cv::Mat> latest_ll2_image_ = std::nullopt;

  void publishImage(const cv::Mat& image, const rclcpp::Time& stamp)
  {
    cv_bridge::CvImage raw_image;
    raw_image.header.stamp = stamp;
    raw_image.header.frame_id = "map";
    raw_image.encoding = "bgr8";
    raw_image.image = image;
    pub_match_image_->publish(*raw_image.toImageMsg());
  }

  void lsdImageCallback(const sensor_msgs::msg::Image& msg)
  {
    cv::Mat lsd_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    if (!latest_ll2_image_.has_value())
      return;

    cv::Mat match_image = lsd_image_ + latest_ll2_image_.value();
    publishImage(match_image, msg.header.stamp);
  }

  void ll2ImageCallback(const sensor_msgs::msg::Image& msg)
  {
    latest_ll2_image_ = cv_bridge::toCvCopy(msg, "rgb8")->image;
  }

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
