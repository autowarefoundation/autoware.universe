#include "camera_ekf_corrector/camera_ekf_corrector.hpp"
#include "camera_ekf_corrector/fast_cos.hpp"
#include "camera_ekf_corrector/logit.hpp"

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
  cost_map_(this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  enable_switch_ = declare_parameter<bool>("enabled_at_first", true);

  // Publication
  pub_image_ = create_publisher<Image>("match_image", 10);
  pub_map_image_ = create_publisher<Image>("cost_map_image", 10);
  pub_marker_ = create_publisher<MarkerArray>("cost_map_range", 10);

  // Subscription
  auto on_lsd = std::bind(&CameraEkfCorrector::on_lsd, this, _1);
  auto on_ll2 = std::bind(&CameraEkfCorrector::on_ll2, this, _1);
  auto on_bounding_box = std::bind(&CameraEkfCorrector::on_bounding_box, this, _1);
  auto on_pose = std::bind(&CameraEkfCorrector::on_pose, this, _1);
  sub_lsd_ = create_subscription<PointCloud2>("lsd_cloud", 10, on_lsd);
  sub_ll2_ = create_subscription<PointCloud2>("ll2_road_marking", 10, on_ll2);
  sub_bounding_box_ = create_subscription<PointCloud2>("ll2_bounding_box", 10, on_bounding_box);
  sub_pose_ = create_subscription<PoseStamped>("particle_pose", 10, on_pose);
}

void CameraEkfCorrector::on_pose(const PoseStamped & msg) { latest_pose_ = msg; }

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

void CameraEkfCorrector::on_lsd(const PointCloud2 & lsd_msg)
{
  common::Timer timer;
  const rclcpp::Time stamp = lsd_msg.header.stamp;

  auto [lsd_cloud, iffy_lsd_cloud] = split_linesegments(lsd_msg);

  const Pose meaned_pose;
  bool publish_measured_pose = true;
  {
    // Check travel distance and publish weights if it is enough long
    Eigen::Vector3f mean_position = common::pose_to_affine(meaned_pose).translation();
    if ((mean_position - last_mean_position_).squaredNorm() > 1) {
      last_mean_position_ = mean_position;
    } else {
      using namespace std::literals::chrono_literals;
      publish_measured_pose = false;
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(), *get_clock(), (1000ms).count(), "Skip weighting because almost same positon");
    }
  }

  cost_map_.set_height(meaned_pose.position.z);

  // if (publish_weighted_particles) {
  //   for (auto & particle : weighted_particles.particles) {
  //     Sophus::SE3f transform = common::pose_to_se3(particle.pose);
  //     LineSegments transformed_lsd = common::transform_linesegments(lsd_cloud, transform);
  //     LineSegments transformed_iffy_lsd = common::transform_linesegments(iffy_lsd_cloud,
  //     transform); transformed_lsd += transformed_iffy_lsd;

  //     float logit = compute_logit(transformed_lsd, transform.translation());
  //     particle.weight = logit_to_prob(logit, 0.01f);
  //   }

  //   if (enable_switch_) {
  //     // this->set_weighted_particle_array(weighted_particles);
  //   }
  // }

  cost_map_.erase_obsolete();  // NOTE:
  pub_marker_->publish(cost_map_.show_map_range());

  if (timer.milli_seconds() > 80) {
    RCLCPP_WARN_STREAM(get_logger(), "on_lsd: " << timer);
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "on_lsd: " << timer);
  }
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
}  // namespace pcdless::ekf_corrector