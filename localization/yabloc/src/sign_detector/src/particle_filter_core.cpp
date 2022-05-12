#include "sign_detector/particle_filter.hpp"
#include "sign_detector/timer.hpp"
#include <opencv4/opencv2/highgui.hpp>

void ParticleFilter::publishTf(const geometry_msgs::msg::PoseStamped& pose)
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

void ParticleFilter::publishImage(const cv::Mat& image, const rclcpp::Time& stamp)
{
  cv_bridge::CvImage raw_image;
  raw_image.header.stamp = stamp;
  raw_image.header.frame_id = "map";
  raw_image.encoding = "bgr8";
  raw_image.image = image;
  pub_match_image_->publish(*raw_image.toImageMsg());
}

cv::Mat ParticleFilter::extractGreenChannel(const cv::Mat& src_image)
{
  std::vector<cv::Mat> one_ch_images;
  cv::split(src_image, one_ch_images);
  return one_ch_images.at(1);
}

void ParticleFilter::syncrhoCallback(const sensor_msgs::msg::Image& msg1, const sensor_msgs::msg::Image& msg2)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "ll2: " << msg1.header.stamp.sec << "." << msg1.header.stamp.nanosec);
  RCLCPP_INFO_STREAM(this->get_logger(), "lsd: " << msg2.header.stamp.sec << "." << msg2.header.stamp.nanosec);
  cv::Mat ll2_image = cv_bridge::toCvCopy(msg1, "bgr8")->image;
  cv::Mat lsd_image = cv_bridge::toCvCopy(msg2, "rgb8")->image;

  // ll2_image = extractGreenChannel(ll2_image);
  // lsd_image = extractGreenChannel(lsd_image);
  // cv::Mat match_image = matchImage(lsd_image, ll2_image);
  cv::Mat match_image = ll2_image + lsd_image;
  publishImage(match_image, msg1.header.stamp);
}

void ParticleFilter::lsdImageCallback(const sensor_msgs::msg::Image& msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "lsd: " << msg.header.stamp.sec << "." << msg.header.stamp.nanosec);
  cv::Mat lsd_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  lsd_image = extractGreenChannel(lsd_image);
  if (!latest_ll2_image_.has_value())
    return;

  cv::Mat match_image = matchImage(lsd_image, latest_ll2_image_.value());
  publishImage(match_image, msg.header.stamp);
}

void ParticleFilter::ll2ImageCallback(const sensor_msgs::msg::Image& msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "ll2: " << msg.header.stamp.sec << "." << msg.header.stamp.nanosec);
  cv::Mat ll2_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  latest_ll2_image_ = extractGreenChannel(ll2_image);
}

cv::Mat ParticleFilter::matchImage(const cv::Mat& query, const cv::Mat& ref)
{
  Timer timer;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilate_size_ + 1, 2 * dilate_size_ + 1), cv::Point(dilate_size_, dilate_size_));
  cv::Mat dst_ref;
  cv::dilate(ref, dst_ref, kernel);
  cv::GaussianBlur(dst_ref, dst_ref, cv::Size(blur_size_, blur_size_), 0, 0);

  cv::Rect roi(cv::Point(200, 400), cv::Size(400, 200));
  cv::Mat temp = query(roi);  // 切り出し画像
  cv::Mat ret;
  cv::matchTemplate(dst_ref, temp, ret, CV_TM_CCORR_NORMED);
  RCLCPP_INFO_STREAM(this->get_logger(), "blur time: " << timer << " ms");
  cv::imshow("show", ret);
  cv::waitKey(1);

  cv::Mat zero = cv::Mat::zeros(query.size(), CV_8UC1);
  cv::Mat merged;
  cv::merge(std::vector<cv::Mat>{query, dst_ref, zero}, merged);


  return merged;
}