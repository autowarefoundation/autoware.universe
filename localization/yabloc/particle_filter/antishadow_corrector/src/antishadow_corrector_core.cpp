#include "antishadow_corrector/antishadow_corrector.hpp"

#include <opencv4/opencv2/imgproc.hpp>
#include <pcdless_common/cv_decompress.hpp>
#include <pcdless_common/extract_line_segments.hpp>
#include <pcdless_common/pose_conversions.hpp>
#include <pcdless_common/timer.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcdless::modularized_particle_filter
{
AntishadowCorrector::AntishadowCorrector()
: AbstCorrector("camera_particle_corrector"),
  score_min_(declare_parameter<float>("score_min", 70)),
  score_max_(declare_parameter<float>("score_max", 128)),
  weight_min_(declare_parameter<float>("weight_min", 0.5)),
  print_statistics_(declare_parameter<bool>("print_statistics", false))
{
  using std::placeholders::_1;
  auto lsd_callback = std::bind(&AntishadowCorrector::on_lsd, this, _1);
  auto ll2_callback = std::bind(&AntishadowCorrector::on_ll2, this, _1);
  auto pose_callback = std::bind(&AntishadowCorrector::on_pose_stamped, this, _1);

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

void AntishadowCorrector::on_pose_stamped(const PoseStamped & msg)
{
  latest_pose_ = common::pose_to_se3(msg.pose);
}

void AntishadowCorrector::on_lsd(const Image & msg)
{
  common::Timer timer;

  cv::Mat lsd_image = common::decompress_to_cv_mat(msg);
  cv::cvtColor(lsd_image, lsd_image, cv::COLOR_BGR2GRAY);  // NOTE: remove redundant channel

  const rclcpp::Time stamp = msg.header.stamp;
  std::optional<ParticleArray> opt_array = this->get_synchronized_particle_array(stamp);
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

  LineSegments cropped_ll2_cloud = common::extract_near_line_segments(ll2_cloud_, *latest_pose_);

  auto normalize = define_normalize_score();
  for (auto & p : weighted_particles.particles) {
    Sophus::SE3f pose = common::pose_to_se3(p.pose);
    LineSegments dst_cloud;
    pcl::transformPointCloudWithNormals(cropped_ll2_cloud, dst_cloud, pose.ivnerse().matrix());
    float score = compute_score(dst_cloud, lsd_image);
    p.weight = normalize(score);
  }

  if (print_statistics_) print_particle_statistics(weighted_particles);

  // NOTE: skip weighting if ego vehicle does not move enought
  auto meaned_pose = mean_pose(weighted_particles);
  Eigen::Vector3f mean_position = common::pose_to_affine(meaned_pose).translation();
  if ((mean_position - last_mean_position_).squaredNorm() > 1) {
    this->set_weighted_particle_array(weighted_particles);
    last_mean_position_ = mean_position;
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Skip weighting because almost same positon");
  }

  RCLCPP_INFO_STREAM(get_logger(), "onLsd() " << timer);
}

void AntishadowCorrector::print_particle_statistics(const ParticleArray & array) const
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

float AntishadowCorrector::compute_score(const LineSegments & src, const cv::Mat & lsd_image) const
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
  if (pixel_count == 0) return 0;
  return score / pixel_count;
}

void AntishadowCorrector::on_ll2(const PointCloud2 & ll2_msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "LL2 cloud is subscribed");
  pcl::fromROSMsg(ll2_msg, ll2_cloud_);
}

std::function<float(float)> AntishadowCorrector::define_normalize_score() const
{
  float k = -std::log(weight_min_);

  return [this, k](float raw_score) -> float {
    float clamped_score = std::clamp(raw_score, this->score_min_, this->score_max_);
    float r = (clamped_score - score_min_) / (score_max_ - score_min_);
    float weight = this->weight_min_ * std::exp(k * r);

    if (!std::isfinite(weight)) {
      throw std::runtime_error("invalid weight is yielded from " + std::to_string(raw_score));
    }
    return weight;
  };
}

}  // namespace pcdless::modularized_particle_filter