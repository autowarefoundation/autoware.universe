#include "segment_accumulator/segment_accumulator.hpp"

#include <opencv2/highgui.hpp>
#include <pcdless_common/cv_decompress.hpp>
#include <pcdless_common/extract_line_segments.hpp>
#include <pcdless_common/pose_conversions.hpp>
#include <pcdless_common/pub_sub.hpp>
#include <pcdless_common/timer.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace pcdless::accumulator
{
SegmentAccumulator::SegmentAccumulator()
: Node("reprojector"),
  min_segment_length_(declare_parameter<float>("min_segment_length", 0.5)),
  default_map_value_(declare_parameter<int>("default_map_value", 00)),
  map_update_interval_(declare_parameter<float>("map_update_interval", 0.5)),
  info_(this),
  tf_subscriber_(this->get_clock())
{
  using std::placeholders::_1;

  // Subscriber
  auto on_lsd = std::bind(&SegmentAccumulator::on_line_segments, this, _1);
  auto on_twist = std::bind(&SegmentAccumulator::on_twist, this, _1);
  auto on_ll2 = std::bind(&SegmentAccumulator::on_ll2, this, _1);
  auto on_pose = std::bind(&SegmentAccumulator::on_pose_stamped, this, _1);

  sub_ll2_ = create_subscription<PointCloud2>("/localization/map/ll2_road_marking", 10, on_ll2);
  sub_pose_stamped_ =
    create_subscription<PoseStamped>("/localization/pf/particle_pose", 10, on_pose);
  sub_lsd_ = create_subscription<PointCloud2>("lsd_cloud", 10, on_lsd);
  sub_twist_ = create_subscription<TwistStamped>("/localization/twist/kalman/twist", 10, on_twist);

  // Publisher
  pub_image_ = create_publisher<Image>("mapping_image", 10);
  pub_rgb_image_ = create_publisher<Image>("rgb_mapping_image", 10);

  histogram_image_ =
    default_map_value_ * cv::Mat::ones(cv::Size(2 * IMAGE_RADIUS, 2 * IMAGE_RADIUS), CV_8UC1);
}

void SegmentAccumulator::on_twist(TwistStamped::ConstSharedPtr msg) { twist_list_.push_back(msg); }

cv::Point2f SegmentAccumulator::cv_pt2(const Eigen::Vector3f & v) const
{
  return {-v.y() / METRIC_PER_PIXEL + IMAGE_RADIUS, -v.x() / METRIC_PER_PIXEL + IMAGE_RADIUS};
}

Eigen::Vector3f SegmentAccumulator::eigen_vec3f(const cv::Point2f & p) const
{
  return {-(p.y - IMAGE_RADIUS) * METRIC_PER_PIXEL, -(p.x - IMAGE_RADIUS) * METRIC_PER_PIXEL, 0};
}

void SegmentAccumulator::define_project_function()
{
  std::optional<Eigen::Affine3f> camera_extrinsic =
    tf_subscriber_(info_.get_frame_id(), "base_link");
  if (!camera_extrinsic.has_value()) return;
  if (info_.is_camera_info_nullopt()) return;
  const Eigen::Vector2i size = info_.size();
  const Eigen::Matrix3f K = info_.intrinsic();
  const Eigen::Matrix3f Kinv = K.inverse();

  const Eigen::Vector3f t = camera_extrinsic->translation();
  const Eigen::Quaternionf q(camera_extrinsic->rotation());

  back_project_func_ = [Kinv, q, t](const Eigen::Vector3f & u) -> std::optional<Eigen::Vector3f> {
    Eigen::Vector3f u3(u.x(), u.y(), 1);
    Eigen::Vector3f u_bearing = (q * Kinv * u3).normalized();
    if (u_bearing.z() > -0.01) return std::nullopt;
    float u_distance = -t.z() / u_bearing.z();
    Eigen::Vector3f v;
    v.x() = t.x() + u_bearing.x() * u_distance;
    v.y() = t.y() + u_bearing.y() * u_distance;
    v.z() = 0;
    return v;
  };

  // Compute visible area as convex hull polygon
  {
    auto check_projectable = [K, q, t, size](const Eigen::Vector3f & u) -> bool {
      Eigen::Vector3f u_from_camera = (q.conjugate() * (u - t));
      if (u_from_camera.z() < 0.01) return false;

      Eigen::Vector3f pixel = K * u_from_camera / u_from_camera.z();
      if (pixel.x() < 0 || pixel.x() > size.x()) return false;
      if (pixel.y() < 0 || pixel.y() > size.y()) return false;
      return true;
    };

    std::vector<cv::Point2i> visible_points;
    for (int c = 0; c < 2 * IMAGE_RADIUS; ++c) {
      for (int r = 0; r < 2 * IMAGE_RADIUS; ++r) {
        cv::Point2i pixel(r, c);
        if (check_projectable(eigen_vec3f(pixel))) visible_points.push_back(pixel);
      }
    }

    visible_area_polygon_.clear();
    cv::convexHull(visible_points, visible_area_polygon_);
  }
}

void SegmentAccumulator::on_line_segments(const PointCloud2 & msg)
{
  common::Timer timer;
  static std::optional<rclcpp::Time> last_stamp{std::nullopt};

  if (last_stamp.has_value()) {
    Sophus::SE3f odom = accumulate_travel_distance(*last_stamp, msg.header.stamp).inverse();
    if (odom.translation().norm() < map_update_interval_) {
      RCLCPP_INFO_STREAM(
        get_logger(),
        "skip accumulation due to not enought travel distance: " << odom.translation().norm());
      return;  // NOTE: Skip map update
    }

    transform_image(odom);
  }
  last_stamp = msg.header.stamp;

  if (!back_project_func_) {
    define_project_function();
    if (!back_project_func_) return;
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr lsd{new pcl::PointCloud<pcl::PointNormal>()};
  pcl::fromROSMsg(msg, *lsd);

  LineSegments::Ptr filted_segments = filt_line_segments_by_ll2(lsd);
  draw(filted_segments);
  pop_obsolete_msg(*last_stamp);

  cv::Mat color_image, histogram_3ch_image;
  cv::applyColorMap(histogram_image_, color_image, cv::COLORMAP_JET);
  cv::cvtColor(histogram_image_, histogram_3ch_image, cv::COLOR_GRAY2BGR);

  common::publish_image(*pub_image_, histogram_3ch_image, msg.header.stamp);
  common::publish_image(*pub_rgb_image_, color_image, msg.header.stamp);

  RCLCPP_INFO_STREAM(get_logger(), "line accumulation: " << timer);
}

SegmentAccumulator::LineSegments::Ptr SegmentAccumulator::filt_line_segments_by_ll2(
  const LineSegments::Ptr & lines)
{
  if (!latest_pose_.has_value()) {
    return lines;
  }
  LineSegments near_ll2 = common::extract_near_line_segments(ll2_cloud_, *latest_pose_);

  // TODO: use transform_cloud() in antishadow_corrector.cpp
  cv::Mat debug_image = cv::Mat::zeros(500, 500, CV_8UC3);
  for (const pcl::PointNormal & pn : near_ll2) {
    Eigen::Vector3f p1 = latest_pose_->inverse() * pn.getVector3fMap();
    Eigen::Vector3f p2 = latest_pose_->inverse() * pn.getNormalVector3fMap();

    cv::line(debug_image, cv_pt2(p1), cv_pt2(p2), cv::Scalar(0, 255, 255));
  }
  cv::imshow("debug", debug_image);
  cv::waitKey(10);

  return lines;
}

void SegmentAccumulator::transform_image(const Sophus::SE3f & odom)
{
  // Transform histogram image

  auto transform = [this, odom](const cv::Point2f & src) -> cv::Point2f {
    Eigen::Vector3f v = eigen_vec3f(src);
    return cv_pt2(odom * v);
  };

  std::vector<cv::Point2f> src_points;
  std::vector<cv::Point2f> dst_points;
  src_points.push_back(cv::Point2f(400, 500));
  src_points.push_back(cv::Point2f(300, 400));
  src_points.push_back(cv::Point2f(500, 400));
  for (const auto & p : src_points) dst_points.push_back(transform(p));

  cv::Mat warp_mat = cv::getAffineTransform(src_points, dst_points);
  cv::warpAffine(
    histogram_image_, histogram_image_, warp_mat, histogram_image_.size(), 1, 0,
    cv::Scalar::all(default_map_value_));
}

void SegmentAccumulator::draw(const LineSegments::Ptr & line_segments)
{
  // Project segment on ground
  cv::Mat increment_image = cv::Mat::zeros(histogram_image_.size(), CV_8UC1);
  for (const auto & ls : *line_segments) {
    std::optional<Eigen::Vector3f> opt1 = back_project_func_(ls.getVector3fMap());
    std::optional<Eigen::Vector3f> opt2 = back_project_func_(ls.getNormalVector3fMap());
    if (!opt1.has_value()) continue;
    if (!opt2.has_value()) continue;
    float length = (opt1.value() - opt2.value()).norm();
    if (length < min_segment_length_) continue;

    cv::line(increment_image, cv_pt2(*opt1), cv_pt2(*opt2), cv::Scalar::all(255), 1);
  }
  histogram_image_ += increment_image;

  // Decrease belief in visible area
  {
    cv::Mat decrease_image = cv::Mat::zeros(histogram_image_.size(), CV_8UC1);
    cv::fillPoly(decrease_image, visible_area_polygon_, cv::Scalar::all(10), 8, 0);
    histogram_image_ -= decrease_image;
  }
}

Sophus::SE3f SegmentAccumulator::accumulate_travel_distance(
  const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp)
{
  // TODO: Honestly, this accumulation does not provide accurate relative pose

  Sophus::SE3f pose;

  rclcpp::Time last_stamp = from_stamp;
  for (auto itr = twist_list_.begin(); itr != twist_list_.end(); ++itr) {
    rclcpp::Time stamp{(*itr)->header.stamp};
    if (stamp < from_stamp) continue;
    if (stamp > to_stamp) break;

    double dt = (stamp - last_stamp).seconds();
    const auto & linear = (*itr)->twist.linear;
    const auto & angular = (*itr)->twist.angular;

    Eigen::Vector3f v(linear.x, linear.y, linear.z);
    Eigen::Vector3f w(angular.x, angular.y, angular.z);
    pose *= Sophus::SE3f{Sophus::SO3f::exp(w * dt), dt * v};
    last_stamp = stamp;
  }

  return pose;
}

void SegmentAccumulator::pop_obsolete_msg(const rclcpp::Time & oldest_stamp)
{
  for (auto itr = twist_list_.begin(); itr != twist_list_.end();) {
    if (rclcpp::Time((*itr)->header.stamp) > oldest_stamp) break;
    itr = twist_list_.erase(itr);
  }
}

void SegmentAccumulator::on_ll2(const PointCloud2 & ll2_msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "LL2 cloud is subscribed");
  pcl::fromROSMsg(ll2_msg, ll2_cloud_);
}

void SegmentAccumulator::on_pose_stamped(const PoseStamped & msg)
{
  latest_pose_ = common::pose_to_se3(msg.pose);
}

}  // namespace pcdless::accumulator
