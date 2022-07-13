#include "common/util.hpp"
#include "particle_filter/camera_corrector.hpp"

#include <opencv4/opencv2/imgproc.hpp>

#include <pcl_conversions/pcl_conversions.h>
namespace particle_filter
{

CameraParticleCorrector::CameraParticleCorrector()
: AbstCorrector("camera_particle_corrector"),
  image_size_(declare_parameter<int>("image_size", 800)),
  max_range_(declare_parameter<float>("max_range", 20.f)),
  gamma_(declare_parameter<float>("gamma", 3.0f)),
  score_offset_(declare_parameter<float>("score_offset", -64.f)),
  max_raw_score_(declare_parameter<float>("max_raw_score", 5000.0)),
  min_prob_(declare_parameter<float>("min_prob", 0.01)),
  far_weight_gain_(declare_parameter<float>("far_weight_gain", 0.001)),
  cost_map_(this->get_logger(), max_range_, image_size_, gamma_)
{
  using std::placeholders::_1;

  auto lsd_callback = std::bind(&CameraParticleCorrector::lsdCallback, this, _1);
  lsd_sub_ = create_subscription<PointCloud2>("/lsd_cloud", 10, lsd_callback);

  auto ll2_callback = std::bind(&CameraParticleCorrector::ll2Callback, this, _1);
  ll2_sub_ = create_subscription<PointCloud2>("/ll2_road_marking", 10, ll2_callback);

  auto pose_callback = std::bind(&CameraParticleCorrector::poseCallback, this, _1);
  pose_sub_ = create_subscription<PoseStamped>("/particle_pose", 10, pose_callback);

  image_pub_ = create_publisher<Image>("/match_image", 10);
  marker_pub_ = create_publisher<MarkerArray>("/cost_map_range", 10);
}

void CameraParticleCorrector::lsdCallback(const sensor_msgs::msg::PointCloud2 & lsd_msg)
{
  const rclcpp::Time stamp = lsd_msg.header.stamp;
  std::optional<ParticleArray> opt_array = this->getSyncronizedParticleArray(stamp);

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
    Eigen::Affine3f transform = util::pose2Affine(particle.pose);
    LineSegment transformed_lsd = transformCloud(lsd_cloud, transform);

    float raw_score = computeScore(transformed_lsd, transform.translation());
    particle.weight = score_convert(raw_score);
  }

  cost_map_.eraseObsolete();

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
      score += gain * (cost_map_.at(p.topRows(2)) + score_offset_);
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