#include "modularized_particle_filter/correction/camera_pose_corrector.hpp"

#include <opencv4/opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

cv::Point2f CameraPoseCorrector::toCvPoint(const Eigen::Vector3f & p)
{
  const cv::Size center(image_size_ / 2, image_size_ / 2);
  cv::Point2f pt;
  pt.x = -p.y() / max_range_ * center.width + center.width;
  pt.y = -p.x() / max_range_ * center.height + 2 * center.height;
  return pt;
};

cv::Mat CameraPoseCorrector::cloud2Image(const pcl::PointCloud<pcl::PointNormal> & cloud)
{
  cv::Mat image = cv::Mat::zeros(cv::Size(image_size_, image_size_), CV_8UC1);

  for (const auto pn : cloud)
    cv::line(
      image, toCvPoint(pn.getVector3fMap()), toCvPoint(pn.getNormalVector3fMap()),
      cv::Scalar::all(255), 2);

  return image;
}

cv::Mat CameraPoseCorrector::visualizePairs(
  const std::vector<std::pair<pcl::PointNormal, pcl::PointNormal>> & pairs)
{
  cv::Mat image = cv::Mat::zeros(cv::Size(image_size_, image_size_), CV_8UC1);
  const cv::Size center(image.cols / 2, image.rows / 2);

  for (const auto & pair : pairs) {
    const auto pn1 = pair.first;
    const auto pn2 = pair.second;
    std::vector<cv::Point> points;
    points.push_back(toCvPoint(pn1.getVector3fMap()));
    points.push_back(toCvPoint(pn1.getNormalVector3fMap()));
    if ((pn1.x > pn1.normal_x) xor (pn2.x > pn2.normal_x)) {
      points.push_back(toCvPoint(pn2.getVector3fMap()));
      points.push_back(toCvPoint(pn2.getNormalVector3fMap()));
    } else {
      points.push_back(toCvPoint(pn2.getNormalVector3fMap()));
      points.push_back(toCvPoint(pn2.getVector3fMap()));
    }
    cv::fillConvexPoly(image, points, cv::Scalar::all(255));
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

void CameraPoseCorrector::synchroCallback(
  const PointCloud2 & lsd_msg, const CloudPose & ll2_msg, const ParticleArray & particles)
{
  rclcpp::Duration dt = rclcpp::Time(lsd_msg.header.stamp) - rclcpp::Time(ll2_msg.header.stamp);
  RCLCPP_INFO_STREAM(this->get_logger(), "dt: " << dt.seconds());

  pcl::PointCloud<pcl::PointNormal> lsd_cloud, ll2_cloud;
  pcl::fromROSMsg(lsd_msg, lsd_cloud);
  pcl::fromROSMsg(ll2_msg.cloud, ll2_cloud);

  cv::Mat lsd_image = cloud2Image(lsd_cloud);
  cv::Mat ll2_image = cloud2Image(ll2_cloud);

  const Eigen::Affine3f base_transform = pose2Affine(ll2_msg.pose);

  // Compute particles' score
  ParticleArray weighted_particles = particles;
  for (auto & p : weighted_particles.particles) p.weight = 1;

  auto distanceSegments = [](const pcl::PointNormal & p1, const pcl::PointNormal & p2) -> float {
    Eigen::Vector3f from = p1.getVector3fMap();
    Eigen::Vector3f t = (p1.getNormalVector3fMap() - from).normalized();
    Eigen::Vector3f d1 = p2.getVector3fMap() - from;
    Eigen::Vector3f d2 = p2.getNormalVector3fMap() - from;
    return (d1.cross(t)).norm() + (d2.cross(t)).norm();
  };

  auto start = std::chrono::system_clock::now();
  // Search nearest neighbor segments
  std::vector<std::pair<pcl::PointNormal, pcl::PointNormal>> paired_segments;
  for (const pcl::PointNormal & pn1 : lsd_cloud) {
    Eigen::Vector3f t1 = (pn1.getNormalVector3fMap() - pn1.getVector3fMap()).normalized();
    float l1 = (pn1.getVector3fMap() - pn1.getNormalVector3fMap()).norm();

    pcl::PointNormal best_pn2;
    float best_score = std::numeric_limits<float>::max();
    for (const pcl::PointNormal & pn2 : ll2_cloud) {
      Eigen::Vector3f t2 = (pn2.getVector3fMap() - pn2.getNormalVector3fMap()).normalized();
      if (std::abs(t1.dot(t2)) < 0.9) continue;

      Eigen::Vector3f d1 = pn2.getVector3fMap() - pn1.getVector3fMap();
      Eigen::Vector3f d2 = pn2.getNormalVector3fMap() - pn1.getVector3fMap();
      float m1 = d1.dot(t1);
      float m2 = d2.dot(t1);
      if (std::min(std::max(m1, m2), l1) < std::max(std::min(m1, m2), 0.f)) continue;

      float score = distanceSegments(pn1, pn2);
      if (score < best_score) {
        best_score = score;
        best_pn2 = pn2;
      }
    }
    if (best_score < score_threshold_) {
      paired_segments.emplace_back(pn1, best_pn2);
    }
  }
  {
    auto dur = std::chrono::system_clock::now() - start;
    long us = std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
    RCLCPP_INFO_STREAM(get_logger(), "time of matching: " << us);
  }

  cv::Mat pair_image = visualizePairs(paired_segments);

  // {
  //   Eigen::Affine3f particle_transform = pose2Affine(max_element->pose);
  //   Eigen::Affine3f transform = base_transform.inverse() * particle_transform;
  //   Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  //   T.topLeftCorner(3, 3) = transform.rotation().transpose();
  //   T.topRightCorner(3, 1) = to_image_coord * scale * transform.translation();
  //   pcl::PointCloud<pcl::PointNormal> dst;
  //   pcl::transformPointCloud(lsd_cloud, dst, T);
  // }

  // Build nice image
  cv::Mat match_image;
  cv::merge(std::vector<cv::Mat>{ll2_image, lsd_image, pair_image}, match_image);

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