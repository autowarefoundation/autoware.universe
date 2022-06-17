#include "common/util.hpp"
#include "particle_filter/camera_corrector.hpp"

#include <opencv4/opencv2/imgproc.hpp>

#include <pcl_conversions/pcl_conversions.h>
namespace particle_filter
{
void CameraParticleCorrector::lsdCallback(const sensor_msgs::msg::PointCloud2 & lsd_msg)
{
  const rclcpp::Time stamp = lsd_msg.header.stamp;
  std::optional<ParticleArray> opt_array = this->getSyncronizedParticleArray(stamp);

  if (!opt_array.has_value()) return;

  float dt = (stamp - opt_array->header.stamp).seconds();
  RCLCPP_INFO_STREAM(get_logger(), "opt_arary stamp delay: " << dt * 1000.f << " ms");

  LineSegment lsd_cloud;
  pcl::fromROSMsg(lsd_msg, lsd_cloud);

  cv::Mat ll2_image;

  auto score_convert = [this, k = -std::log(min_prob_) / 2.f](float raw) -> float {
    raw = std::clamp(raw, -this->max_raw_score_, this->max_raw_score_);
    return this->min_prob_ * std::exp(k * (raw / this->max_raw_score_ + 1));
  };

  ParticleArray weighted_particles = opt_array.value();

  for (auto & particle : weighted_particles.particles) {
    Eigen::Affine3f transform = util::pose2Affine(particle.pose);
    LineSegment transformed_lsd = transformCloud(lsd_cloud, transform);

    float raw_score = computeScore(transformed_lsd, ll2_image);
    particle.weight = score_convert(raw_score);
  }

  this->setWeightedParticleArray(weighted_particles);
  marker_pub_->publish(cost_map_.showMapRange());
}

void CameraParticleCorrector::poseCallback(const PoseStamped & msg)
{
  util::publishImage(*image_pub_, cost_map_.getMapImage(msg.pose), msg.header.stamp);
}

void CameraParticleCorrector::ll2Callback(const PointCloud2 & ll2_msg)
{
  LineSegment ll2_cloud;
  pcl::fromROSMsg(ll2_msg, ll2_cloud);
  cost_map_.setCloud(ll2_cloud);
  RCLCPP_INFO_STREAM(get_logger(), "Set LL2 cloud into Hierarchical cost map");
}

float CameraParticleCorrector::computeScore(const LineSegment & lsd_cloud, cv::Mat & ll2_image)
{
  float score = 0;
  for (const pcl::PointNormal & pn1 : lsd_cloud) {
    Eigen::Vector3f t1 = (pn1.getNormalVector3fMap() - pn1.getVector3fMap()).normalized();
    float l1 = (pn1.getVector3fMap() - pn1.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < l1; distance += 0.1f) {
      Eigen::Vector3f p = pn1.getVector3fMap() + t1 * distance;

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