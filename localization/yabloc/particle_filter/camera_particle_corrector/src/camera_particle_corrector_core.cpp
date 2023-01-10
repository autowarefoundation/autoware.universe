#include "camera_particle_corrector/camera_particle_corrector.hpp"

#include <opencv4/opencv2/imgproc.hpp>
#include <pcdless_common/color.hpp>
#include <pcdless_common/pose_conversions.hpp>
#include <pcdless_common/pub_sub.hpp>
#include <pcdless_common/timer.hpp>
#include <pcdless_common/transform_linesegments.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace pcdless::modularized_particle_filter
{
namespace
{
struct FastCosSin
{
  FastCosSin(int bin = 90)
  {
    for (int i = 0; i < bin + 1; ++i) {
      cos_.push_back(std::cos(i * M_PI / 180.f));
    }
  }
  float cos(float deg) const
  {
    while (deg < 0) {
      deg += 360;
    }
    while (deg > 360) {
      deg -= 360;
    }
    if (deg < 90) {
      return cos_.at(int(deg));
    } else if (deg < 180) {
      return -cos_.at(int(180 - deg));
    } else if (deg < 270) {
      return -cos_.at(int(deg - 180));
    } else {
      return cos_.at(int(360 - deg));
    }
  }

  float sin(float deg) const { return cos(deg - 90.f); }

private:
  std::vector<float> cos_;
};

FastCosSin fast_math;
}  // namespace

CameraParticleCorrector::CameraParticleCorrector()
: AbstCorrector("camera_particle_corrector"),
  score_offset_(declare_parameter<float>("score_offset", -64.f)),
  max_raw_score_(declare_parameter<float>("max_raw_score", 5000.0)),
  min_prob_(declare_parameter<float>("min_prob", 0.01)),
  far_weight_gain_(declare_parameter<float>("far_weight_gain", 0.001)),
  cost_map_(this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  enable_switch_ = declare_parameter<bool>("enabled_at_first", true);

  // Publication
  pub_image_ = create_publisher<Image>("match_image", 10);
  pub_map_image_ = create_publisher<Image>("cost_map_image", 10);
  pub_marker_ = create_publisher<MarkerArray>("cost_map_range", 10);
  pub_scored_cloud_ = create_publisher<PointCloud2>("scored_cloud", 10);
  pub_string_ = create_publisher<String>("state_string", 10);

  // Subscription
  auto on_lsd = std::bind(&CameraParticleCorrector::on_lsd, this, _1);
  auto on_ll2 = std::bind(&CameraParticleCorrector::on_ll2, this, _1);
  auto on_unmapped_area = std::bind(&CameraParticleCorrector::on_unmapped_area, this, _1);
  auto on_pose = std::bind(&CameraParticleCorrector::on_pose, this, _1);
  sub_lsd_ = create_subscription<PointCloud2>("lsd_cloud", 10, on_lsd);
  sub_ll2_ = create_subscription<PointCloud2>("ll2_road_marking", 10, on_ll2);
  sub_unmapped_area_ = create_subscription<PointCloud2>("ll2_polygon", 10, on_unmapped_area);
  sub_pose_ = create_subscription<PoseStamped>("particle_pose", 10, on_pose);

  auto on_service = std::bind(&CameraParticleCorrector::on_service, this, _1, _2);
  switch_service_ = create_service<SetBool>("/switch", on_service);

  // Timer callback
  auto on_timer = std::bind(&CameraParticleCorrector::on_timer, this);
  timer_ =
    rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(5).period(), std::move(on_timer));

  score_converter_ = [this, k = -std::log(min_prob_) / 2.f](float raw) -> float {
    raw = std::clamp(raw, -this->max_raw_score_, this->max_raw_score_);
    return this->min_prob_ * std::exp(k * (raw / this->max_raw_score_ + 1));
  };
}

void CameraParticleCorrector::on_pose(const PoseStamped & msg) { latest_pose_ = msg; }

void CameraParticleCorrector::on_service(
  SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response)
{
  enable_switch_ = request->data;
  response->success = true;
  if (enable_switch_)
    RCLCPP_INFO_STREAM(get_logger(), "camera_corrector is enabled");
  else
    RCLCPP_INFO_STREAM(get_logger(), "camera_corrector is disabled");
}

void CameraParticleCorrector::on_unmapped_area(const PointCloud2 & msg)
{
  // NOTE: Under construction
  // TODO: This does not handle multiple polygons
  pcl::PointCloud<pcl::PointXYZ> ll2_polygon;
  pcl::fromROSMsg(msg, ll2_polygon);
  cost_map_.set_unmapped_area(ll2_polygon);
  RCLCPP_INFO_STREAM(get_logger(), "Set unmapped-area into Hierarchical cost map");
}

std::pair<CameraParticleCorrector::LineSegments, CameraParticleCorrector::LineSegments>
CameraParticleCorrector::split_linesegments(const PointCloud2 & msg)
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

void CameraParticleCorrector::on_lsd(const PointCloud2 & lsd_msg)
{
  common::Timer timer;
  const rclcpp::Time stamp = lsd_msg.header.stamp;
  std::optional<ParticleArray> opt_array = this->get_synchronized_particle_array(stamp);
  if (!opt_array.has_value()) {
    return;
  }

  const rclcpp::Duration dt = (stamp - opt_array->header.stamp);
  if (std::abs(dt.seconds()) > 0.1) {
    const std::string text = "Timestamp gap between image and particles is LARGE ";
    RCLCPP_WARN_STREAM(get_logger(), text << dt.seconds());
  }

  auto [lsd_cloud, iffy_lsd_cloud] = split_linesegments(lsd_msg);
  ParticleArray weighted_particles = opt_array.value();

  bool publish_weighted_particles = true;
  const Pose meaned_pose = mean_pose(weighted_particles);
  {
    // Check travel distance and publish weights if it is enough long
    Eigen::Vector3f mean_position = common::pose_to_affine(meaned_pose).translation();
    if ((mean_position - last_mean_position_).squaredNorm() > 1) {
      last_mean_position_ = mean_position;
    } else {
      using namespace std::literals::chrono_literals;
      publish_weighted_particles = false;
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(), *get_clock(), (1000ms).count(), "Skip weighting because almost same positon");
    }
  }

  cost_map_.set_height(meaned_pose.position.z);

  if (publish_weighted_particles) {
    for (auto & particle : weighted_particles.particles) {
      Sophus::SE3f transform = common::pose_to_se3(particle.pose);
      LineSegments transformed_lsd = common::transform_linesegments(lsd_cloud, transform);
      LineSegments transformed_iffy_lsd = common::transform_linesegments(iffy_lsd_cloud, transform);
      transformed_lsd += transformed_iffy_lsd;

      float raw_score = compute_score(transformed_lsd, transform.translation());
      particle.weight = score_converter_(raw_score);
    }

    if (enable_switch_) {
      this->set_weighted_particle_array(weighted_particles);
    }
  }

  cost_map_.erase_obsolete();  // NOTE:
  pub_marker_->publish(cost_map_.show_map_range());

  // DEBUG: just visualization
  {
    Pose meaned_pose = mean_pose(weighted_particles);
    Sophus::SE3f transform = common::pose_to_se3(meaned_pose);

    pcl::PointCloud<pcl::PointXYZI> cloud =
      evaluate_cloud(common::transform_linesegments(lsd_cloud, transform), transform.translation());
    pcl::PointCloud<pcl::PointXYZI> iffy_cloud = evaluate_cloud(
      common::transform_linesegments(iffy_lsd_cloud, transform), transform.translation());

    pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;
    float max_score = 0;
    for (const auto p : cloud) {
      max_score = std::max(max_score, std::abs(p.intensity));
    }
    for (const auto p : cloud) {
      pcl::PointXYZRGB rgb;
      rgb.getVector3fMap() = p.getVector3fMap();
      rgb.rgba = common::color_scale::blue_red(0.5 + p.intensity / max_score * 0.5);
      rgb_cloud.push_back(rgb);
    }
    for (const auto p : iffy_cloud) {
      pcl::PointXYZRGB rgb;
      rgb.getVector3fMap() = p.getVector3fMap();
      rgb.rgba = common::Color(0, 0.8, 0, 0.5f);
      rgb_cloud.push_back(rgb);
    }

    common::publish_cloud(*pub_scored_cloud_, rgb_cloud, lsd_msg.header.stamp);
  }

  if (timer.milli_seconds() > 80) {
    RCLCPP_WARN_STREAM(get_logger(), "on_lsd: " << timer);
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "on_lsd: " << timer);
  }

  // Publish status as string
  {
    String msg;
    std::stringstream ss;
    ss << "-- Camera particle corrector --" << std::endl;
    ss << (enable_switch_ ? "ENABLED" : "disabled") << std::endl;
    ss << "time: " << timer << std::endl;
    msg.data = ss.str();
    pub_string_->publish(msg);
  }
}

void CameraParticleCorrector::on_timer()
{
  if (latest_pose_.has_value())
    common::publish_image(
      *pub_map_image_, cost_map_.get_map_image(latest_pose_->pose), latest_pose_->header.stamp);
}

void CameraParticleCorrector::on_ll2(const PointCloud2 & ll2_msg)
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

float CameraParticleCorrector::compute_score(
  const LineSegments & lsd_cloud, const Eigen::Vector3f & self_position)
{
  float score = 0;
  for (const LineSegment & pn : lsd_cloud) {
    const Eigen::Vector3f tangent = (pn.getNormalVector3fMap() - pn.getVector3fMap()).normalized();
    const float length = (pn.getVector3fMap() - pn.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < length; distance += 0.1f) {
      Eigen::Vector3f p = pn.getVector3fMap() + tangent * distance;

      // NOTE: Close points are prioritized
      float squared_norm = (p - self_position).topRows(2).squaredNorm();
      float gain = exp(-far_weight_gain_ * squared_norm);

      cv::Vec2b v2 = cost_map_.at2(p.topRows(2));
      if (pn.label == 0) {
        score += 0.5 * gain * (abs_cos(tangent, v2[1]) * v2[0] + score_offset_);
      } else {
        score += gain * (abs_cos(tangent, v2[1]) * v2[0] + score_offset_);
      }
    }
  }
  return score;
}

pcl::PointCloud<pcl::PointXYZI> CameraParticleCorrector::evaluate_cloud(
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

      cv::Vec2b v2 = cost_map_.at2(p.topRows(2));
      float score = gain * (abs_cos(tangent, v2[1]) * v2[0] + score_offset_);

      pcl::PointXYZI xyzi(score);
      xyzi.getVector3fMap() = p;
      cloud.push_back(xyzi);
    }
  }
  return cloud;
}
}  // namespace pcdless::modularized_particle_filter