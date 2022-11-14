#include "antishadow_corrector/antishadow_corrector.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <vml_common/cv_decompress.hpp>
#include <vml_common/pose_conversions.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace modularized_particle_filter
{
AntishadowCorrector::AntishadowCorrector() : AbstCorrector("camera_particle_corrector")
{
  using std::placeholders::_1;
  auto lsd_callback = std::bind(&AntishadowCorrector::onLsd, this, _1);
  auto ll2_callback = std::bind(&AntishadowCorrector::onLl2, this, _1);
  sub_lsd_ = create_subscription<Image>("mapping_image", 10, lsd_callback);
  sub_ll2_ =
    create_subscription<PointCloud2>("/localization/map/ll2_road_marking", 10, ll2_callback);
}

cv::Point2f AntishadowCorrector::cv_pt2(const Eigen::Vector3f & v) const
{
  return {-v.y() / METRIC_PER_PIXEL + IMAGE_RADIUS, -v.x() / METRIC_PER_PIXEL + IMAGE_RADIUS};
}

void AntishadowCorrector::onLsd(const Image & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "LSD map is subscribed");

  const cv::Mat lsd_image = vml_common::decompress2CvMat(msg);

  const rclcpp::Time stamp = msg.header.stamp;
  std::optional<ParticleArray> opt_array = this->getSynchronizedParticleArray(stamp);
  if (!opt_array.has_value()) return;

  const auto dt = (stamp - opt_array->header.stamp);
  if (std::abs(dt.seconds()) > 0.1) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Timestamp gap between image and particles is LARGE " << dt.seconds());
    return;
  }

  if (ll2_cloud_.empty()) {
    RCLCPP_WARN_STREAM(get_logger(), "LL2 cloud is empty");
    return;
  }

  ParticleArray weighted_particles = opt_array.value();
  for (auto & p : weighted_particles.particles) {
    Sophus::SE3f pose = vml_common::pose2Se3(p.pose);
    auto dst_cloud = transformCloud(ll2_cloud_, pose.inverse());
    p.weight = computeScore(dst_cloud, lsd_image);
    break;
  }

  // TODO: skip weighting if ego vehicle does not move enought

  this->setWeightedParticleArray(weighted_particles);
}

float AntishadowCorrector::computeScore(const LineSegment & src, const cv::Mat & lsd_image) const
{
  cv::Rect rect(0, 0, lsd_image.cols, lsd_image.rows);
  cv::Mat ll2_image = cv::Mat::zeros(lsd_image.size(), CV_8UC3);

  for (const pcl::PointNormal & pn : src) {
    Eigen::Vector3f t1 = (pn.getNormalVector3fMap() - pn.getVector3fMap()).normalized();
    float l1 = (pn.getVector3fMap() - pn.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < l1; distance += 0.1f) {
      Eigen::Vector3f p = pn.getVector3fMap() + t1 * distance;
      cv::Point2f px = cv_pt2(p);
      if (rect.contains(px)) ll2_image.at<cv::Vec3b>(px) = cv::Vec3b(0, 255, 255);
      // int score = cost_map_.at2(p.topRows(2));
    }
  }

  cv::imshow("ll2", ll2_image);
  cv::waitKey(1);

  return 0.0f;
}

void AntishadowCorrector::onLl2(const PointCloud2 & ll2_msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "LL2 cloud is subscribed");
  pcl::fromROSMsg(ll2_msg, ll2_cloud_);
}

AntishadowCorrector::LineSegment AntishadowCorrector::transformCloud(
  const LineSegment & src, const Sophus::SE3f & transform) const
{
  LineSegment dst;
  dst.reserve(src.size());
  for (const pcl::PointNormal & pn : src) {
    pcl::PointNormal dst_pn;
    dst_pn.getVector3fMap() = transform * pn.getVector3fMap();
    dst_pn.getNormalVector3fMap() = transform * pn.getNormalVector3fMap();
    dst.push_back(dst_pn);
  }
  return dst;
}

}  // namespace modularized_particle_filter