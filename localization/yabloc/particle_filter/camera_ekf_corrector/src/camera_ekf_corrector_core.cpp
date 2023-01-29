#include "camera_ekf_corrector/camera_ekf_corrector.hpp"
#include "camera_ekf_corrector/fast_cos.hpp"
#include "camera_ekf_corrector/logit.hpp"
#include "camera_ekf_corrector/sampling.hpp"

#include <opencv4/opencv2/imgproc.hpp>
#include <pcdless_common/color.hpp>
#include <pcdless_common/pose_conversions.hpp>
#include <pcdless_common/pub_sub.hpp>
#include <pcdless_common/timer.hpp>
#include <pcdless_common/transform_linesegments.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace pcdless::ekf_corrector
{
FastCosSin fast_math;

CameraEkfCorrector::CameraEkfCorrector()
: Node("camera_particle_corrector"),
  far_weight_gain_(declare_parameter<float>("far_weight_gain", 0.001)),
  logit_gain_(declare_parameter<float>("logit_gain", 0.1)),
  cost_map_(this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Publication
  pub_image_ = create_publisher<Image>("match_image", 10);
  pub_pose_cov_ = create_publisher<PoseCovStamped>("output/pose_with_covariance", 10);
  pub_debug_pose_cov_ = create_publisher<PoseCovStamped>("debug/pose_with_covariance", 10);
  pub_markers_ = create_publisher<MarkerArray>("yielded_marker", 10);
  pub_scored_cloud_ = create_publisher<PointCloud2>("debug_scored_cloud", 10);

  // Subscription
  auto on_lsd = std::bind(&CameraEkfCorrector::on_lsd, this, _1);
  auto on_ll2 = std::bind(&CameraEkfCorrector::on_ll2, this, _1);
  auto on_bounding_box = std::bind(&CameraEkfCorrector::on_bounding_box, this, _1);
  auto on_pose_cov = std::bind(&CameraEkfCorrector::on_pose_cov, this, _1);
  sub_lsd_ = create_subscription<PointCloud2>("lsd_cloud", 10, on_lsd);
  sub_ll2_ = create_subscription<PointCloud2>("ll2_road_marking", 10, on_ll2);
  sub_bounding_box_ = create_subscription<PointCloud2>("ll2_bounding_box", 10, on_bounding_box);
  sub_pose_cov_ =
    create_subscription<PoseCovStamped>("input/pose_with_covariance", 10, on_pose_cov);
  sub_initialpose_ = create_subscription<PoseCovStamped>(
    "/localization/initializer/rectified/initialpose", 10,
    [this](const PoseCovStamped &) -> void { this->enable_switch_ = true; });

  auto on_height = [this](const Float32 & height) { this->latest_height_ = height; };
  sub_height_ = create_subscription<Float32>("input/height", 10, on_height);
}

void CameraEkfCorrector::on_pose_cov(const PoseCovStamped & msg) { pose_buffer_.push_back(msg); }

void CameraEkfCorrector::on_bounding_box(const PointCloud2 & msg)
{
  // NOTE: Under construction
  pcl::PointCloud<pcl::PointXYZL> ll2_bounding_box;
  pcl::fromROSMsg(msg, ll2_bounding_box);
  cost_map_.set_bounding_box(ll2_bounding_box);
  RCLCPP_INFO_STREAM(get_logger(), "Set bounding box into cost map");
}

std::pair<CameraEkfCorrector::LineSegments, CameraEkfCorrector::LineSegments>
CameraEkfCorrector::split_linesegments(const PointCloud2 & msg)
{
  LineSegments all_lsd_cloud;
  pcl::fromROSMsg(msg, all_lsd_cloud);
  LineSegments reliable_cloud, iffy_cloud;
  {
    for (const auto & p : all_lsd_cloud) {
      if (p.label == 0)
        iffy_cloud.push_back(p);
      else
        reliable_cloud.push_back(p);
    }
  }

  auto [good_cloud, bad_cloud] = filt(iffy_cloud);
  {
    cv::Mat debug_image = cv::Mat::zeros(800, 800, CV_8UC3);
    auto draw = [&debug_image](LineSegments & cloud, cv::Scalar color) -> void {
      for (const auto & line : cloud) {
        const Eigen::Vector3f p1 = line.getVector3fMap();
        const Eigen::Vector3f p2 = line.getNormalVector3fMap();
        cv::line(debug_image, cv2pt(p1), cv2pt(p2), color, 2);
      }
    };

    draw(reliable_cloud, cv::Scalar(0, 0, 255));
    draw(good_cloud, cv::Scalar(0, 255, 0));
    draw(bad_cloud, cv::Scalar(100, 100, 100));
    common::publish_image(*pub_image_, debug_image, msg.header.stamp);
  }

  return {reliable_cloud, good_cloud};
}

std::optional<CameraEkfCorrector::PoseCovStamped> CameraEkfCorrector::get_synchronized_pose(
  const rclcpp::Time & stamp)
{
  constexpr double acceptable_max_delay = 2.0f;

  auto itr = pose_buffer_.begin();
  while (itr != pose_buffer_.end()) {
    rclcpp::Duration dt = rclcpp::Time(itr->header.stamp) - stamp;
    if (dt.seconds() < -acceptable_max_delay)
      pose_buffer_.erase(itr++);
    else
      break;
  }

  if (pose_buffer_.empty()) {
    RCLCPP_WARN_STREAM(get_logger(), "sychronized pose are requested but buffer is empty");
    return std::nullopt;
  }

  auto comp = [stamp](PoseCovStamped & x1, PoseCovStamped & x2) -> bool {
    auto dt1 = rclcpp::Time(x1.header.stamp) - stamp;
    auto dt2 = rclcpp::Time(x2.header.stamp) - stamp;
    return std::abs(dt1.seconds()) < std::abs(dt2.seconds());
  };
  return *std::min_element(pose_buffer_.begin(), pose_buffer_.end(), comp);
}

void CameraEkfCorrector::on_lsd(const PointCloud2 & lsd_msg)
{
  using namespace std::literals::chrono_literals;
  common::Timer timer;

  if (pose_buffer_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(), "pose_buffer is empty so skip on_lsd()");
    return;
  }

  if (!enable_switch_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(), "skip until first initialpose is published");
    return;
  }

  const rclcpp::Time stamp = lsd_msg.header.stamp;
  auto opt_synched_pose = get_synchronized_pose(stamp);

  // TEMP:
  if (opt_synched_pose.has_value()) {
    auto dur = rclcpp::Time(opt_synched_pose->header.stamp) - stamp;
    RCLCPP_INFO_STREAM(get_logger(), "Success to sync: " << dur.seconds());
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "Failed to sync: pose buffer has " << pose_buffer_.size());
    return;
  }

  auto [lsd_cloud, iffy_lsd_cloud] = split_linesegments(lsd_msg);
  // iffy_lsd_cloud.clear(); // TODO:

  // cost_map_.set_height(opt_synched_pose->pose.pose.position.z);

  PoseCovStamped estimated_pose =
    estimate_pose_with_covariance(opt_synched_pose.value(), lsd_cloud, iffy_lsd_cloud);

  {
    Sophus::SE3f transform = common::pose_to_se3(opt_synched_pose->pose.pose);
    pcl::PointCloud<pcl::PointXYZI> cloud =
      evaluate_cloud(common::transform_linesegments(lsd_cloud, transform), transform.translation());

    pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;

    for (const auto p : cloud) {
      pcl::PointXYZRGB rgb;
      rgb.getVector3fMap() = p.getVector3fMap();
      rgb.rgba = common::color_scale::blue_red(p.intensity);
      rgb_cloud.push_back(rgb);
    }
    common::publish_cloud(*pub_scored_cloud_, rgb_cloud, lsd_msg.header.stamp);
  }

  // NOTE: Skip weighting if travel distance is too small
  {
    Eigen::Vector3f position = common::pose_to_se3(opt_synched_pose->pose.pose).translation();
    static Eigen::Vector3f last_position = Eigen::Vector3f::Zero();
    const float travel_distance = (position - last_position).norm();
    if (travel_distance > 1) {
      pub_pose_cov_->publish(estimated_pose);
      pub_debug_pose_cov_->publish(estimated_pose);
      last_position = position;
    } else {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 2000, "Skip weighting because almost same positon");
    }
  }

  cost_map_.erase_obsolete();  // NOTE: Don't foreget erasing

  if (timer.milli_seconds() > 80) {
    RCLCPP_WARN_STREAM(get_logger(), "on_lsd: " << timer);
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "on_lsd: " << timer);
  }
}

CameraEkfCorrector::PoseCovStamped CameraEkfCorrector::estimate_pose_with_covariance(
  const PoseCovStamped & init, const LineSegments & lsd_cloud, const LineSegments & iffy_lsd_cloud)
{
  ParticleArray particles;

  // Yield pose candidates from covariance
  Eigen::Matrix2d xy_cov;
  xy_cov << init.pose.covariance[0], init.pose.covariance[1], init.pose.covariance[6],
    init.pose.covariance[7];
  double theta_cov = init.pose.covariance[35];
  const double base_theta =
    2 * std::atan2(init.pose.pose.orientation.z, init.pose.pose.orientation.w);

  NormalDistribution2d nrand2d(xy_cov);

  constexpr int N = 500;
  for (int i = 0; i < N; i++) {
    auto [prob, xy] = nrand2d();
    Particle particle;
    particle.pose = init.pose.pose;
    particle.pose.position.x += xy.x();
    particle.pose.position.y += xy.y();
    particle.pose.orientation.w = 1;
    double theta = base_theta + nrand(theta_cov);
    particle.pose.orientation.w = std::cos(theta / 2.);
    particle.pose.orientation.z = std::sin(theta / 2.);
    particle.pose.orientation.x = 0;
    particle.pose.orientation.y = 0;
    particles.particles.push_back(particle);
  }

  // Find weights for every pose candidates
  for (auto & particle : particles.particles) {
    Sophus::SE3f transform = common::pose_to_se3(particle.pose);
    LineSegments transformed_lsd = common::transform_linesegments(lsd_cloud, transform);
    LineSegments transformed_iffy_lsd = common::transform_linesegments(iffy_lsd_cloud, transform);
    transformed_lsd += transformed_iffy_lsd;

    float logit = compute_logit(transformed_lsd, transform.translation());
    particle.weight = logit_to_prob(logit, logit_gain_);
  }

  // visualize
  publish_visualize_markers(particles);

  // Compute optimal distribution
  const MeanResult result = compile_distribution(particles);
  PoseCovStamped output = init;
  output.pose.pose = result.pose_;
  output.pose.pose.position.z = latest_height_.data;

  for (int i = 0; i < 3; ++i) {
    output.pose.covariance[6 * i + 0] = result.cov_xyz_(i, 0);
    output.pose.covariance[6 * i + 1] = result.cov_xyz_(i, 1);
    output.pose.covariance[6 * i + 2] = result.cov_xyz_(i, 2);
  }
  output.pose = debayes_distribution(output.pose, init.pose);

  output.pose.covariance[6 * 2 + 2] = 0.04;  // Var(z)
  output.pose.covariance[6 * 5 + 5] = result.cov_theta_;

  {
    auto in_pos = init.pose.pose.position;
    auto out_pos = output.pose.pose.position;
    auto out_ori = output.pose.pose.orientation;
    std::cout << "input: " << in_pos.x << " " << in_pos.y << " " << in_pos.z << std::endl;
    std::cout << "output: " << out_pos.x << " " << out_pos.y << " " << out_pos.z << std::endl;
  }
  return output;
}

void CameraEkfCorrector::publish_visualize_markers(const ParticleArray & particles)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto minmax_weight = std::minmax_element(
    particles.particles.begin(), particles.particles.end(),
    [](const Particle & a, const Particle & b) -> bool { return a.weight < b.weight; });

  // float min = minmax_weight.first->weight;
  // float max = minmax_weight.second->weight;
  // max = std::max(max, min + 1e-7f);
  float min = 0.f;
  float max = 1.f;
  auto boundWeight = [min, max](float raw) -> float { return (raw - min) / (max - min); };

  int id = 0;
  for (const Particle & p : particles.particles) {
    visualization_msgs::msg::Marker marker;
    marker.frame_locked = true;
    marker.header.frame_id = "map";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.scale.x = 0.3;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color = common::color_scale::rainbow(boundWeight(p.weight));
    marker.color.a = 0.8;
    marker.pose.orientation = p.pose.orientation;
    marker.pose.position.x = p.pose.position.x;
    marker.pose.position.y = p.pose.position.y;
    marker.pose.position.z = p.pose.position.z;
    marker_array.markers.push_back(marker);
  }

  pub_markers_->publish(marker_array);
}

void CameraEkfCorrector::on_ll2(const PointCloud2 & ll2_msg)
{
  pcl::PointCloud<pcl::PointNormal> ll2_cloud;
  pcl::fromROSMsg(ll2_msg, ll2_cloud);
  cost_map_.set_cloud(ll2_cloud);
  RCLCPP_INFO_STREAM(get_logger(), "Set LL2 cloud into Hierarchical cost map");
}

float abs_cos(const Eigen::Vector3f & t, float deg)
{
  // NOTE: use pre-computed table for std::cos & std::sin
  Eigen::Vector2f x(t.x(), t.y());
  Eigen::Vector2f y(fast_math.cos(deg), fast_math.sin(deg));
  x.normalize();
  return std::abs(x.dot(y));
}

float CameraEkfCorrector::compute_logit(
  const LineSegments & lsd_cloud, const Eigen::Vector3f & self_position)
{
  float logit = 0;
  for (const LineSegment & pn : lsd_cloud) {
    const Eigen::Vector3f tangent = (pn.getNormalVector3fMap() - pn.getVector3fMap()).normalized();
    const float length = (pn.getVector3fMap() - pn.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < length; distance += 0.1f) {
      Eigen::Vector3f p = pn.getVector3fMap() + tangent * distance;

      // NOTE: Close points are prioritized
      float squared_norm = (p - self_position).topRows(2).squaredNorm();
      float gain = exp(-far_weight_gain_ * squared_norm);  // 0 < gain < 1

      const CostMapValue v3 = cost_map_.at(p.topRows(2));

      if (v3.unmapped) {
        // logit does not change if target pixel is unmapped
        continue;
      }
      if (pn.label == 0) {  // posteriori
        logit += 0.2f * gain * (abs_cos(tangent, v3.angle) * v3.intensity - 0.5f);
      } else {  // apriori
        logit += gain * (abs_cos(tangent, v3.angle) * v3.intensity - 0.5f);
      }
    }
  }
  return logit;
}

pcl::PointCloud<pcl::PointXYZI> CameraEkfCorrector::evaluate_cloud(
  const LineSegments & lsd_cloud, const Eigen::Vector3f & self_position)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (const LineSegment & pn : lsd_cloud) {
    Eigen::Vector3f tangent = (pn.getNormalVector3fMap() - pn.getVector3fMap()).normalized();
    float length = (pn.getVector3fMap() - pn.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < length; distance += 0.1f) {
      Eigen::Vector3f p = pn.getVector3fMap() + tangent * distance;

      // NOTE: Close points are prioritized
      float squared_norm = (p - self_position).topRows(2).squaredNorm();
      float gain = std::exp(-far_weight_gain_ * squared_norm);

      CostMapValue v3 = cost_map_.at(p.topRows(2));
      float logit = 0;
      if (!v3.unmapped) logit = gain * (abs_cos(tangent, v3.angle) * v3.intensity - 0.5f);

      pcl::PointXYZI xyzi(logit_to_prob(logit, 10.f));
      xyzi.getVector3fMap() = p;
      cloud.push_back(xyzi);
    }
  }
  return cloud;
}

}  // namespace pcdless::ekf_corrector