#include "camera_particle_corrector/camera_particle_corrector.hpp"

#include <opencv4/opencv2/imgproc.hpp>
#include <vml_common/color.hpp>
#include <vml_common/pose_conversions.hpp>
#include <vml_common/pub_sub.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace modularized_particle_filter
{

CameraParticleCorrector::CameraParticleCorrector()
: AbstCorrector("camera_particle_corrector"),
  score_offset_(declare_parameter<float>("score_offset", -64.f)),
  max_raw_score_(declare_parameter<float>("max_raw_score", 5000.0)),
  min_prob_(declare_parameter<float>("min_prob", 0.01)),
  far_weight_gain_(declare_parameter<float>("far_weight_gain", 0.001)),
  cost_map_(this)
{
  using std::placeholders::_1;

  auto lsd_callback = std::bind(&CameraParticleCorrector::onLsd, this, _1);
  sub_lsd_ = create_subscription<PointCloud2>("lsd_cloud", 10, lsd_callback);

  auto ll2_callback = std::bind(&CameraParticleCorrector::onLl2, this, _1);
  sub_ll2_ = create_subscription<PointCloud2>("ll2_road_marking", 10, ll2_callback);

  auto unmapped_area_callback = std::bind(&CameraParticleCorrector::onUnmappedArea, this, _1);
  sub_unmapped_area_ = create_subscription<PointCloud2>("ll2_polygon", 10, unmapped_area_callback);

  auto pose_callback = std::bind(&CameraParticleCorrector::onPose, this, _1);
  sub_pose_ = create_subscription<PoseStamped>("particle_pose", 10, pose_callback);

  pub_image_ = create_publisher<Image>("match_image", 10);
  pub_marker_ = create_publisher<MarkerArray>("cost_map_range", 10);
  pub_scored_cloud_ = create_publisher<PointCloud2>("scored_cloud", 10);

  // Timer callback
  auto cb_timer = std::bind(&CameraParticleCorrector::onTimer, this);
  timer_ =
    rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(5).period(), std::move(cb_timer));
}

void CameraParticleCorrector::onPose(const PoseStamped & msg) { latest_pose_ = msg; }

void CameraParticleCorrector::onUnmappedArea(const PointCloud2 & msg)
{
  // TODO: This does not handle multiple polygons
  pcl::PointCloud<pcl::PointXYZ> ll2_polygon;
  pcl::fromROSMsg(msg, ll2_polygon);
  cost_map_.setUnmappedArea(ll2_polygon);
  RCLCPP_INFO_STREAM(get_logger(), "Set unmapped-area into Hierarchical cost map");
}

void CameraParticleCorrector::onLsd(const PointCloud2 & lsd_msg)
{
  const rclcpp::Time stamp = lsd_msg.header.stamp;
  std::optional<ParticleArray> opt_array = this->getSynchronizedParticleArray(stamp);

  if (!opt_array.has_value()) return;

  auto dt = (stamp - opt_array->header.stamp);
  if (std::abs(dt.seconds()) > 0.1)
    RCLCPP_WARN_STREAM(
      get_logger(), "Timestamp gap between image and particles is LARGE " << dt.seconds());

  LineSegment lsd_cloud;
  pcl::fromROSMsg(lsd_msg, lsd_cloud);

  auto score_convert = [this, k = -std::log(min_prob_) / 2.f](float raw) -> float {
    raw = std::clamp(raw, -this->max_raw_score_, this->max_raw_score_);
    return this->min_prob_ * std::exp(k * (raw / this->max_raw_score_ + 1));
  };

  ParticleArray weighted_particles = opt_array.value();

  for (auto & particle : weighted_particles.particles) {
    Eigen::Affine3f transform = vml_common::pose2Affine(particle.pose);
    LineSegment transformed_lsd = transformCloud(lsd_cloud, transform);

    float raw_score = computeScore(transformed_lsd, transform.translation());
    particle.weight = score_convert(raw_score);
  }

  cost_map_.eraseObsolete();

  // NOTE:
  Pose mean_pose = modularized_particle_filter::meanPose(weighted_particles);
  Eigen::Vector3f mean_position = vml_common::pose2Affine(mean_pose).translation();
  if ((mean_position - last_mean_position_).squaredNorm() > 1) {
    this->setWeightedParticleArray(weighted_particles);
    last_mean_position_ = mean_position;
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "Skip weighting because almost same positon");
  }

  pub_marker_->publish(cost_map_.showMapRange());

  // DEBUG: just visualization
  {
    Pose mean_pose = modularized_particle_filter::meanPose(weighted_particles);
    Eigen::Affine3f transform = vml_common::pose2Affine(mean_pose);
    LineSegment transformed_lsd = transformCloud(lsd_cloud, transform);
    pcl::PointCloud<pcl::PointXYZI> cloud = evaluateCloud(transformed_lsd, transform.translation());

    pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;
    float max_score = 0;
    for (const auto p : cloud) {
      max_score = std::max(max_score, std::abs(p.intensity));
    }
    for (const auto p : cloud) {
      pcl::PointXYZRGB rgb;
      rgb.getVector3fMap() = p.getVector3fMap();
      rgb.rgba = vml_common::color_scale::blueRed(0.5 + p.intensity / max_score * 0.5);
      rgb_cloud.push_back(rgb);
    }

    vml_common::publishCloud(*pub_scored_cloud_, rgb_cloud, lsd_msg.header.stamp);
  }
}

void CameraParticleCorrector::onTimer()
{
  if (latest_pose_.has_value())
    vml_common::publishImage(
      *pub_image_, cost_map_.getMapImage(latest_pose_->pose), latest_pose_->header.stamp);
}

void CameraParticleCorrector::onLl2(const PointCloud2 & ll2_msg)
{
  LineSegment ll2_cloud;
  pcl::fromROSMsg(ll2_msg, ll2_cloud);
  cost_map_.setCloud(ll2_cloud);
  RCLCPP_INFO_STREAM(get_logger(), "Set LL2 cloud into Hierarchical cost map");
}

float absCos(const Eigen::Vector3f & t, float deg)
{
  Eigen::Vector2f x(t.x(), t.y());
  Eigen::Vector2f y(std::cos(deg * M_PI / 180.0), std::sin(deg * M_PI / 180.0));
  x.normalize();
  return std::abs(x.dot(y));
}

float CameraParticleCorrector::computeScore(
  const LineSegment & lsd_cloud, const Eigen::Vector3f & self_position)
{
  float score = 0;
  for (const pcl::PointNormal & pn1 : lsd_cloud) {
    Eigen::Vector3f t1 = (pn1.getNormalVector3fMap() - pn1.getVector3fMap()).normalized();
    float l1 = (pn1.getVector3fMap() - pn1.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < l1; distance += 0.1f) {
      Eigen::Vector3f p = pn1.getVector3fMap() + t1 * distance;

      // NOTE: Close points are prioritized
      float squared_norm = (p - self_position).topRows(2).squaredNorm();
      float gain = std::exp(-far_weight_gain_ * squared_norm);

      cv::Vec2b v2 = cost_map_.at2(p.topRows(2));
      score += gain * (absCos(t1, v2[1]) * v2[0] + score_offset_);
    }
  }
  return score;
}

pcl::PointCloud<pcl::PointXYZI> CameraParticleCorrector::evaluateCloud(
  const LineSegment & lsd_cloud, const Eigen::Vector3f & self_position)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (const pcl::PointNormal & pn1 : lsd_cloud) {
    Eigen::Vector3f t1 = (pn1.getNormalVector3fMap() - pn1.getVector3fMap()).normalized();
    float l1 = (pn1.getVector3fMap() - pn1.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < l1; distance += 0.1f) {
      Eigen::Vector3f p = pn1.getVector3fMap() + t1 * distance;

      // NOTE: Close points are prioritized
      float squared_norm = (p - self_position).topRows(2).squaredNorm();
      float gain = std::exp(-far_weight_gain_ * squared_norm);

      cv::Vec2b v2 = cost_map_.at2(p.topRows(2));
      float score = gain * (absCos(t1, v2[1]) * v2[0] + score_offset_);

      pcl::PointXYZI xyzi(score);
      xyzi.getVector3fMap() = p;
      cloud.push_back(xyzi);
    }
  }
  return cloud;
}

CameraParticleCorrector::LineSegment CameraParticleCorrector::transformCloud(
  const LineSegment & src, const Eigen::Affine3f & transform) const
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