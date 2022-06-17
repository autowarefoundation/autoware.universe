#include "common/util.hpp"
#include "particle_filter/camera_corrector.hpp"

#include <opencv4/opencv2/imgproc.hpp>

#include <pcl_conversions/pcl_conversions.h>
namespace particle_filter
{
cv::Point2f CameraParticleCorrector::toCvPoint(const Eigen::Vector3f & p)
{
  const cv::Size center(image_size_ / 2, image_size_ / 2);
  cv::Point2f pt;
  pt.x = -p.y() / max_range_ * center.width + center.width;
  pt.y = -p.x() / max_range_ * center.height + center.height;
  return pt;
}

void CameraParticleCorrector::lsdCallback(const sensor_msgs::msg::PointCloud2 & lsd_msg)
{
  const rclcpp::Time stamp = lsd_msg.header.stamp;
  std::optional<ParticleArray> opt_array = this->getSyncronizedParticleArray(stamp);

  if (!opt_array.has_value()) return;

  float dt = (stamp - opt_array->header.stamp).seconds();
  RCLCPP_INFO_STREAM(get_logger(), "opt_arary stamp delay: " << dt * 1000.f << " ms");

  LineSegment lsd_cloud;
  pcl::fromROSMsg(lsd_msg, lsd_cloud);
  const Eigen::Affine3f base_transform = util::pose2Affine(latest_cloud_with_pose_->pose);

  cv::Mat ll2_image;
  // cv::Mat rgb_ll2_image;

  // cv::applyColorMap(ll2_image, rgb_ll2_image, cv::COLORMAP_JET);

  auto score_convert = [this, k = -std::log(min_prob_) / 2.f](float raw) -> float {
    raw = std::clamp(raw, -this->max_raw_score_, this->max_raw_score_);
    return this->min_prob_ * std::exp(k * (raw / this->max_raw_score_ + 1));
  };

  ParticleArray weighted_particles = opt_array.value();

  for (auto & particle : weighted_particles.particles) {
    Eigen::Affine3f transform = base_transform.inverse() * util::pose2Affine(particle.pose);
    LineSegment transformed_lsd = transformCloud(lsd_cloud, transform);

    float raw_score = computeScore(transformed_lsd, ll2_image);
    particle.weight = score_convert(raw_score);

    // cv::circle(rgb_ll2_image, toCvPoint(transform.translation()), 2, cv::Scalar::all(255), -1);
  }

  // util::publishImage(*image_pub_, rgb_ll2_image, stamp);
  this->setWeightedParticleArray(weighted_particles);
  marker_pub_->publish(cost_map_.showMapRange());
}

void CameraParticleCorrector::ll2Callback(const PointCloud2 & ll2_msg)
{
  LineSegment ll2_cloud;
  pcl::fromROSMsg(ll2_msg, ll2_cloud);
  cost_map_.setCloud(ll2_cloud);
  RCLCPP_INFO_STREAM(get_logger(), "Set LL2 cloud into Hierarchical cost map");
}

// OBSL:
// cv::Mat CameraParticleCorrector::buildLl2Image(const LineSegment & cloud)
// {
//   cv::Mat image = 255 - cv::Mat::ones(cv::Size(image_size_, image_size_), CV_8UC1);
//   for (const auto pn : cloud) {
//     cv::line(
//       image, toCvPoint(pn.getVector3fMap()), toCvPoint(pn.getNormalVector3fMap()),
//       cv::Scalar::all(0), 1);
//   }
//   cv::Mat distance;
//   cv::distanceTransform(image, distance, cv::DIST_L2, 3);
//   cv::threshold(distance, distance, 100, 100, cv::THRESH_TRUNC);
//   distance.convertTo(distance, CV_8UC1, -2.55, 255);
//   return gamma_converter(distance);
// }

float CameraParticleCorrector::computeScore(const LineSegment & lsd_cloud, cv::Mat & ll2_image)
{
  float score = 0;
  // const cv::Rect rect(0, 0, ll2_image.cols, ll2_image.rows);
  for (const pcl::PointNormal & pn1 : lsd_cloud) {
    Eigen::Vector3f t1 = (pn1.getNormalVector3fMap() - pn1.getVector3fMap()).normalized();
    float l1 = (pn1.getVector3fMap() - pn1.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < l1; distance += 0.1f) {
      Eigen::Vector3f p = pn1.getVector3fMap() + t1 * distance;

      // OBSL:
      // cv::Point2f px = toCvPoint(p);
      // if (rect.contains(px)) score += ll2_image.at<uchar>(px) + score_offset_;

      // TODO:
      score += cost_map_.at(p.topRows(2)) + score_offset_;
    }
  }
  return score;
}

CameraParticleCorrector::LineSegment CameraParticleCorrector::transformCloud(
  const LineSegment & src, const Eigen::Affine3f & transform)
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

}  // namespace particle_filter