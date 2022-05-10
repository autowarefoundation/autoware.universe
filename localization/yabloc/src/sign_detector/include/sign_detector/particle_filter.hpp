#ifndef PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER_HPP_

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
    this->declare_parameter<int>("blur_size", blur_size_);
    this->declare_parameter<int>("dilate_size", dilate_size_);
    this->get_parameter("pose_topic", pose_topic);
    this->get_parameter("blur_size", blur_size_);
    this->get_parameter("dilate_size", dilate_size_);

    sub_ll2_image_ = this->create_subscription<sensor_msgs::msg::Image>("/ll2_image", 10, std::bind(&ParticleFilter::ll2ImageCallback, this, std::placeholders::_1));
    sub_lsd_image_ = this->create_subscription<sensor_msgs::msg::Image>("/projected_image", 10, std::bind(&ParticleFilter::lsdImageCallback, this, std::placeholders::_1));

    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&ParticleFilter::poseCallback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pub_match_image_ = this->create_publisher<sensor_msgs::msg::Image>("/match_image", 10);
  }

private:
  const std::string frame_id_, child_frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ll2_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_lsd_image_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::optional<cv::Mat> latest_ll2_image_ = std::nullopt;
  int blur_size_, dilate_size_;

  cv::Mat matchImage(const cv::Mat& query, const cv::Mat& ref);

  void publishImage(const cv::Mat& image, const rclcpp::Time& stamp);
  void publishTf(const geometry_msgs::msg::PoseStamped& pose);

  void lsdImageCallback(const sensor_msgs::msg::Image& msg);
  void ll2ImageCallback(const sensor_msgs::msg::Image& msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped& msg) { publishTf(msg); }

  cv::Mat extractGreenChannel(const cv::Mat& src_image);
};
#endif  // PARTICLE_FILTER_HPP_