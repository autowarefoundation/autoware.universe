#include "modularized_particle_filter/correction/camera_pose_corrector.hpp"

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

void CameraPoseCorrector::syncrhoCallback(
  const PointCloud2 & msg1, const CloudPose & msg2, const ParticleArray & particles)
{
  // RCLCPP_INFO_STREAM(this->get_logger(), "ll2: " << rclcpp::Time(msg1.header.stamp).seconds());
  // RCLCPP_INFO_STREAM(this->get_logger(), "lsd: " << rclcpp::Time(msg2.header.stamp).seconds());

  cv::Mat lsd_image = cloud2Image(msg1);
  cv::Mat ll2_image = cloud2Image(msg2.cloud);
  pcl::PointCloud<pcl::PointXYZ> lsd_cloud;
  pcl::fromROSMsg(msg1, lsd_cloud);

  const Eigen::Affine3f base_transform = pose2Affine(msg2.pose);

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
    for (const auto & p : dst) {
      cv::Point2i uv(p.x, p.y);
      if (!uv.inside(cv::Rect(0, 0, 800, 800))) continue;
      sum_weight += ll2_image.at<uchar>(uv);
    }
    particle.weight = sum_weight;
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
  cv::merge(std::vector<cv::Mat>{ll2_image, lsd_image, zero_image}, match_image);

  // Publish
  publishImage(match_image, msg1.header.stamp);
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