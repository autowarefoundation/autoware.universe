#include "antishadow_corrector/antishadow_corrector.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <vml_common/cv_decompress.hpp>
#include <vml_common/pose_conversions.hpp>
#include <vml_common/timer.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace modularized_particle_filter
{
AntishadowCorrector::AntishadowCorrector()
: AbstCorrector("camera_particle_corrector"),
  score_min_(declare_parameter<float>("score_min", 70)),
  score_max_(declare_parameter<float>("score_max", 128)),
  weight_min_(declare_parameter<float>("weight_min", 0.5))
{
  using std::placeholders::_1;
  auto lsd_callback = std::bind(&AntishadowCorrector::onLsd, this, _1);
  auto ll2_callback = std::bind(&AntishadowCorrector::onLl2, this, _1);
  auto pose_callback = std::bind(&AntishadowCorrector::onPoseStamped, this, _1);

  sub_lsd_ = create_subscription<Image>("mapping_image", 10, lsd_callback);
  sub_ll2_ =
    create_subscription<PointCloud2>("/localization/map/ll2_road_marking", 10, ll2_callback);
  sub_pose_stamped_ =
    create_subscription<PoseStamped>("/localization/pf/particle_pose", 10, pose_callback);
}

cv::Point2f AntishadowCorrector::cv_pt2(const Eigen::Vector3f & v) const
{
  return {-v.y() / METRIC_PER_PIXEL + IMAGE_RADIUS, -v.x() / METRIC_PER_PIXEL + IMAGE_RADIUS};
}

void AntishadowCorrector::onPoseStamped(const PoseStamped & msg)
{
  latest_pose_ = vml_common::pose2Se3(msg.pose);
}

void AntishadowCorrector::onLsd(const Image & msg)
{
  Timer timer;

  cv::Mat lsd_image = vml_common::decompress2CvMat(msg);
  cv::cvtColor(lsd_image, lsd_image, cv::COLOR_BGR2GRAY);  // NOTE: remove redundant channel

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

  if (!latest_pose_.has_value()) {
    RCLCPP_WARN_STREAM(get_logger(), "latest pose is nullopt");
    return;
  }

  ParticleArray weighted_particles = opt_array.value();

  LineSegments cropped_ll2_cloud = cropLineSegments(ll2_cloud_, *latest_pose_);

  auto normalize = defineNormalizeScore();
  for (auto & p : weighted_particles.particles) {
    Sophus::SE3f pose = vml_common::pose2Se3(p.pose);
    auto dst_cloud = transformCloud(cropped_ll2_cloud, pose.inverse());
    float score = computeScore(dst_cloud, lsd_image);
    p.weight = normalize(score);
  }

  printParticleStatistics(weighted_particles);

  // TODO: skip weighting if ego vehicle does not move enought

  this->setWeightedParticleArray(weighted_particles);
  RCLCPP_INFO_STREAM(get_logger(), "onLsd() " << timer);
}

void AntishadowCorrector::printParticleStatistics(const ParticleArray & array) const
{
  const int N = array.particles.size();
  std::vector<float> weights;
  weights.reserve(N);

  float sum = 0, sum2 = 0;
  for (const auto & p : array.particles) {
    weights.push_back(p.weight);
    sum += p.weight;
    sum2 += p.weight * p.weight;
  }
  std::sort(weights.begin(), weights.end());

  const float min = weights.front();
  const float max = weights.back();
  const float median = weights.at(N / 2);
  const float mean = sum / N;
  const float var = sum2 / N - mean * mean;
  const float sigma = std::sqrt(var);

  std::cout << "min: " << min << std::endl;
  std::cout << "max: " << max << std::endl;
  std::cout << "median: " << median << std::endl;
  std::cout << "mean: " << mean << std::endl;
  std::cout << "sigma: " << sigma << std::endl;
}

float AntishadowCorrector::computeScore(const LineSegments & src, const cv::Mat & lsd_image) const
{
  cv::Rect rect(0, 0, lsd_image.cols, lsd_image.rows);

  float score = 0;
  int pixel_count = 0;
  for (const pcl::PointNormal & pn : src) {
    Eigen::Vector3f t1 = (pn.getNormalVector3fMap() - pn.getVector3fMap()).normalized();
    float l1 = (pn.getVector3fMap() - pn.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < l1; distance += 0.1f) {
      Eigen::Vector3f p = pn.getVector3fMap() + t1 * distance;
      cv::Point2f px = cv_pt2(p);
      if (!rect.contains(px)) continue;

      score += lsd_image.at<unsigned char>(px);
      pixel_count++;
    }
  }

  return score / pixel_count;
}

void AntishadowCorrector::onLl2(const PointCloud2 & ll2_msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "LL2 cloud is subscribed");
  pcl::fromROSMsg(ll2_msg, ll2_cloud_);
}

AntishadowCorrector::LineSegments AntishadowCorrector::transformCloud(
  const LineSegments & src, const Sophus::SE3f & transform) const
{
  LineSegments dst;
  dst.reserve(src.size());
  for (const pcl::PointNormal & pn : src) {
    pcl::PointNormal dst_pn;
    dst_pn.getVector3fMap() = transform * pn.getVector3fMap();
    dst_pn.getNormalVector3fMap() = transform * pn.getNormalVector3fMap();
    dst.push_back(dst_pn);
  }
  return dst;
}

AntishadowCorrector::LineSegments AntishadowCorrector::cropLineSegments(
  const LineSegments & src, const Sophus::SE3f & transform) const
{
  Eigen::Vector3f pose_vector = transform.translation();

  // Compute distance between pose and linesegment of linestring
  auto checkIntersection = [this, pose_vector](const pcl::PointNormal & pn) -> bool {
    const float max_range = 40;

    const Eigen::Vector3f from = pn.getVector3fMap() - pose_vector;
    const Eigen::Vector3f to = pn.getNormalVector3fMap() - pose_vector;

    Eigen::Vector3f tangent = to - from;
    if (tangent.squaredNorm() < 1e-3f) {
      return from.norm() < 1.42 * max_range;
    }

    float inner = from.dot(tangent);
    float mu = std::clamp(inner / tangent.squaredNorm(), -1.0f, 0.0f);
    Eigen::Vector3f nearest = from - tangent * mu;
    return nearest.norm() < 1.42 * max_range;
  };

  LineSegments dst;
  for (const pcl::PointNormal & pn : src) {
    if (checkIntersection(pn)) {
      dst.push_back(pn);
    }
  }
  return dst;
}

std::function<float(float)> AntishadowCorrector::defineNormalizeScore() const
{
  float k = -std::log(weight_min_);

  return [this, k](float score) -> float {
    score = std::clamp(score, this->score_min_, this->score_max_);
    float r = (score - score_min_) / (score_max_ - score_min_);
    return this->weight_min_ * std::exp(k * r);
  };
}

}  // namespace modularized_particle_filter