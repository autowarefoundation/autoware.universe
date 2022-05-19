#include "modularized_particle_filter/correction/camera_pose_corrector.hpp"

#include <opencv4/opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

cv::Mat cloud2Image(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(msg, cloud);

  cv::Mat image = cv::Mat::zeros(cv::Size(800, 800), CV_8UC1);
  for (const auto p : cloud) {
    image.at<uchar>(p.y, p.x) = 255;
  }
  return image;
}

Eigen::Affine3f pose2Affine(const geometry_msgs::msg::Pose & pose)
{
  const auto pos = pose.position;
  const auto ori = pose.orientation;
  Eigen::Translation3f t(pos.x, pos.y, pos.z);
  Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
  return t * q;
}

cv::Mat CameraPoseCorrector::dilateImage(const cv::Mat & image)
{
  cv::Mat dst;
  int size = 2 * 3 + 1;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
  cv::dilate(image, dst, kernel);
  cv::GaussianBlur(
    dst, dst, cv::Size(2 * size + 1, 2 * size + 1), 0, 0, cv::BorderTypes::BORDER_DEFAULT);
  return dst;
}

void CameraPoseCorrector::syncrhoCallback(
  const PointCloud2 & lsd_msg, const CloudPose & ll2_msg, const ParticleArray & particles)
{
  rclcpp::Duration dt = rclcpp::Time(lsd_msg.header.stamp) - rclcpp::Time(ll2_msg.header.stamp);
  RCLCPP_INFO_STREAM(this->get_logger(), "dt: " << dt.seconds());

  cv::Mat lsd_image = cloud2Image(lsd_msg);
  cv::Mat ll2_image = cloud2Image(ll2_msg.cloud);
  pcl::PointCloud<pcl::PointXYZ> lsd_cloud;
  pcl::fromROSMsg(lsd_msg, lsd_cloud);

  ll2_image = dilateImage(ll2_image);

  const Eigen::Affine3f base_transform = pose2Affine(ll2_msg.pose);

  cv::Mat zero_image = cv::Mat::zeros(ll2_image.size(), CV_8UC1);

  // Compute particles' score
  ParticleArray weighted_particles = particles;
  for (auto & particle : weighted_particles.particles) {
    particle.weight = 0;
  }

  const float max_range = 20.0f;
  const float image_size = 800.0f;
  const float scale = image_size / max_range;

  Eigen::Matrix3f to_image_coord;
  to_image_coord << 0, -1, 0, -1, 0, 0, 0, 0, 1;

  for (auto & particle : weighted_particles.particles) {
    Eigen::Affine3f particle_transform = pose2Affine(particle.pose);
    Eigen::Affine3f transform = base_transform.inverse() * particle_transform;

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.topLeftCorner(3, 3) = transform.rotation().transpose();
    T.topRightCorner(3, 1) = to_image_coord * scale * transform.translation();
    pcl::PointCloud<pcl::PointXYZ> dst;
    pcl::transformPointCloud(lsd_cloud, dst, T);

    float sum_weight = 0;
    int count_on_plane = 0;
    for (const auto & p : dst) {
      cv::Point2i uv(p.x, p.y);
      count_on_plane++;
      if (!uv.inside(cv::Rect(0, 0, 800, 800))) continue;
      sum_weight += ll2_image.at<uchar>(uv) / 255.0f;
    }
    particle.weight = sum_weight / static_cast<float>(count_on_plane);
    particle.weight = std::max(particle.weight, 0.2f);
  }

  auto max_element = std::max_element(
    weighted_particles.particles.begin(), weighted_particles.particles.end(),
    [](const auto & p, const auto & q) -> bool { return p.weight < q.weight; });
  {
    Eigen::Affine3f particle_transform = pose2Affine(max_element->pose);
    Eigen::Affine3f transform = base_transform.inverse() * particle_transform;

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.topLeftCorner(3, 3) = transform.rotation().transpose();
    T.topRightCorner(3, 1) = to_image_coord * scale * transform.translation();
    pcl::PointCloud<pcl::PointXYZ> dst;
    pcl::transformPointCloud(lsd_cloud, dst, T);
    for (const auto & p : dst) {
      cv::Point2i uv(p.x, p.y);
      if (!uv.inside(cv::Rect(0, 0, 800, 800))) continue;
      zero_image.at<uchar>(uv) = 255;
    }
  }

  // Build nice image
  cv::Mat match_image;
  // cv::merge(std::vector<cv::Mat>{ll2_image, lsd_image, zero_image}, match_image);
  cv::applyColorMap(ll2_image, match_image, cv::COLORMAP_JET);
  cv::cvtColor(zero_image, zero_image, cv::COLOR_GRAY2BGR);
  cv::addWeighted(match_image, 0.7, zero_image, 1.0, 1, match_image);

  // Publish
  publishImage(match_image, lsd_msg.header.stamp);
  weighted_particle_pub_->publish(weighted_particles);
}

void CameraPoseCorrector::publishImage(const cv::Mat & image, const rclcpp::Time & stamp)
{
  cv_bridge::CvImage raw_image;
  raw_image.header.stamp = stamp;
  raw_image.header.frame_id = "map";
  raw_image.encoding = "bgr8";
  raw_image.image = image;
  image_pub_->publish(*raw_image.toImageMsg());
}