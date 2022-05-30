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
}

cv::Mat CameraPoseCorrector::cloud2Image(const pcl::PointCloud<pcl::PointNormal> & cloud)
{
  cv::Mat image = cv::Mat::zeros(cv::Size(image_size_, image_size_), CV_8UC1);

  for (const auto pn : cloud)
    cv::line(
      image, toCvPoint(pn.getVector3fMap()), toCvPoint(pn.getNormalVector3fMap()),
      cv::Scalar::all(255), 2);

  return image;
}

cv::Mat CameraPoseCorrector::buildLl2Image(const pcl::PointCloud<pcl::PointNormal> & cloud)
{
  cv::Mat image = 255 - cv::Mat::ones(cv::Size(image_size_, image_size_), CV_8UC1);

  for (const auto pn : cloud)
    cv::line(
      image, toCvPoint(pn.getVector3fMap()), toCvPoint(pn.getNormalVector3fMap()),
      cv::Scalar::all(0), 1);

  cv::Mat distance;
  cv::distanceTransform(image, distance, cv::DIST_L2, 3);
  cv::threshold(distance, distance, 100, 100, cv::THRESH_TRUNC);
  distance.convertTo(distance, CV_8UC1, -2.55, 255);
  cv::LUT(distance, lut_, distance);

  // cv::Mat show_distance;
  // cv::applyColorMap(distance, show_distance, cv::COLORMAP_JET);
  // cv::imshow("distance",distance);
  // cv::waitKey(1);
  return distance;
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

pcl::PointCloud<pcl::PointNormal> transformCloud(
  const pcl::PointCloud<pcl::PointNormal> & cloud, const Eigen::Affine3f & transform)
{
  pcl::PointCloud<pcl::PointNormal> dst;
  dst.reserve(cloud.size());
  for (const pcl::PointNormal & pn : cloud) {
    pcl::PointNormal dst_pn;
    dst_pn.getVector3fMap() = transform * pn.getVector3fMap();
    dst_pn.getNormalVector3fMap() = transform * pn.getNormalVector3fMap();
    dst.push_back(dst_pn);
  }

  return dst;
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

float distanceSegments(const pcl::PointNormal & p1, const pcl::PointNormal & p2)
{
  Eigen::Vector3f from = p1.getVector3fMap();
  Eigen::Vector3f t = (p1.getNormalVector3fMap() - from).normalized();
  Eigen::Vector3f d1 = p2.getVector3fMap() - from;
  Eigen::Vector3f d2 = p2.getNormalVector3fMap() - from;
  return (d1.cross(t)).norm() + (d2.cross(t)).norm();
}

pcl::PointCloud<pcl::PointNormal> CameraPoseCorrector::rejectOutsideLines(
  const pcl::PointCloud<pcl::PointNormal> & cloud)
{
  pcl::PointCloud<pcl::PointNormal> dst;

  auto checkInside = [](const Eigen::Vector3f & p) -> bool {
    if (std::abs(p.y()) > 5) return false;
    if (p.x() > 40) return false;
    return true;
  };

  for (const pcl::PointNormal & line : cloud) {
    Eigen::Vector3f from = line.getVector3fMap();
    Eigen::Vector3f to = line.getNormalVector3fMap();
    if (checkInside(from) && checkInside(to)) dst.push_back(line);
  }
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

  lsd_cloud = rejectOutsideLines(lsd_cloud);

  cv::Mat lsd_image = cloud2Image(lsd_cloud);
  cv::Mat ll2_image = buildLl2Image(ll2_cloud);

  const Eigen::Affine3f base_transform = pose2Affine(ll2_msg.pose);

  // Compute particles' score
  ParticleArray weighted_particles = particles;
  // ParticleArray weighted_particles;
  // for (int i = 0; i < 20; i++) {
  //   for (int j = 0; j < 10; j++) {
  //     modularized_particle_filter_msgs::msg::Particle p;
  //     p.pose = ll2_msg.pose;
  //     p.pose.position.x -= (i - 10);
  //     p.pose.position.y -= (j - 5);
  //     weighted_particles.particles.push_back(p);
  //   }
  // }

  // Classify LL2 segments by angle
  std::unordered_map<int, std::vector<pcl::PointNormal>> angle_and_lines;
  for (const pcl::PointNormal & pn : ll2_cloud) {
    Eigen::Vector3f t = pn.getNormalVector3fMap() - pn.getVector3fMap();
    float angle = std::atan2(t.y(), t.x()) * 180.f / M_PI;
    angle_and_lines[static_cast<int>(angle)].push_back(pn);
  }

  // Search nearest neighbor segments
  auto start = std::chrono::system_clock::now();
  // std::vector<std::pair<pcl::PointNormal, pcl::PointNormal>> paired_segments;
  // cv::Mat pair_image;
  // for (auto & particle : weighted_particles.particles) {
  //   Eigen::Affine3f transform = base_transform.inverse() * pose2Affine(particle.pose);
  //   pcl::PointCloud<pcl::PointNormal> transformed_lsd = transformCloud(lsd_cloud, transform);
  //   pair_image = cloud2Image(transformed_lsd);
  //   auto [a, b] = computeScore(transformed_lsd, angle_and_lines);
  //   particle.weight = a;
  //   paired_segments = b;
  // }
  float max_score = -1;
  for (auto & particle : weighted_particles.particles) {
    Eigen::Affine3f transform = base_transform.inverse() * pose2Affine(particle.pose);
    pcl::PointCloud<pcl::PointNormal> transformed_lsd = transformCloud(lsd_cloud, transform);
    particle.weight = computeScore(transformed_lsd, ll2_image);

    max_score = std::max(max_score, particle.weight);
  }
  {
    auto dur = std::chrono::system_clock::now() - start;
    long ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    RCLCPP_INFO_STREAM(
      get_logger(), "time: " << ms << " ll2: " << ll2_cloud.size() << " lsd: " << lsd_cloud.size()
                             << " max_score: " << max_score);
  }

  // Build nice image
  cv::Mat match_image;
  cv::merge(std::vector<cv::Mat>{ll2_image, lsd_image, ll2_image}, match_image);

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

std::pair<float, std::vector<std::pair<pcl::PointNormal, pcl::PointNormal>>>
CameraPoseCorrector::computeScore(
  const pcl::PointCloud<pcl::PointNormal> & lsd_cloud,
  std::unordered_map<int, std::vector<pcl::PointNormal>> & angle_and_lines)
{
  float score = 0;
  std::vector<std::pair<pcl::PointNormal, pcl::PointNormal>> paired_segments;

  for (const pcl::PointNormal & pn1 : lsd_cloud) {
    Eigen::Vector3f t1 = (pn1.getNormalVector3fMap() - pn1.getVector3fMap()).normalized();
    float l1 = (pn1.getVector3fMap() - pn1.getNormalVector3fMap()).norm();

    pcl::PointNormal best_pn2;
    float best_score = std::numeric_limits<float>::max();

    for (const auto [angle_deg, cloud] : angle_and_lines) {
      // First, exclude clusters whose angles do not match to surpress computation cost
      float angle = static_cast<float>(angle_deg) * M_PI / 180.f;
      Eigen::Vector3f t2(std::cos(angle), std::sin(angle), 0);
      if (std::abs(t1.dot(t2)) < 0.95) continue;

      for (const pcl::PointNormal & pn2 : cloud) {
        // If the foot grated to the line segment does not share the line segment, exclude it
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
    }
    if (best_score < score_threshold_) {
      score += 1.0f;
      paired_segments.push_back({pn1, best_pn2});
    }
  }
  return {score, paired_segments};
}

float CameraPoseCorrector::computeScore(
  const pcl::PointCloud<pcl::PointNormal> & lsd_cloud, const cv::Mat & ll2_image)
{
  float score = 0;

  const cv::Rect rect(0, 0, ll2_image.cols, ll2_image.rows);
  for (const pcl::PointNormal & pn1 : lsd_cloud) {
    Eigen::Vector3f t1 = (pn1.getNormalVector3fMap() - pn1.getVector3fMap()).normalized();
    float l1 = (pn1.getVector3fMap() - pn1.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < l1; distance += 0.1f) {
      Eigen::Vector3f p = pn1.getVector3fMap() + t1 * distance;

      cv::Point2f px = toCvPoint(p);
      if (rect.contains(px)) score += ll2_image.at<uchar>(px);
    }
  }
  return score;
}