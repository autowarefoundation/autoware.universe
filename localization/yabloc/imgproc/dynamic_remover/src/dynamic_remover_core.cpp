#include "dynamic_remover/dynamic_remover.hpp"

#include <opencv4/opencv2/imgproc.hpp>
#include <pcdless_common/cv_decompress.hpp>
#include <pcdless_common/pub_sub.hpp>
#include <pcdless_common/timer.hpp>

#include <boost/range/combine.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace pcdless::dynamic_remover
{
DynamicRemover::DynamicRemover()
: Node("reprojector"),
  min_segment_length_(declare_parameter<float>("min_segment_length", 0.5)),
  polygon_thick_(declare_parameter<int>("polygon_thic", 3)),
  gap_threshold_(declare_parameter<float>("gap_threshold", 150)),
  search_iteration_max_(declare_parameter<int>("search_iteration_max", 4)),
  backward_frame_interval_(declare_parameter<int>("backward_frame_interval", 4)),
  info_(this),
  tf_subscriber_(this->get_clock()),
  synchro_subscriber_(this, "/sensing/camera/undistorted/image_raw", "lsd_cloud")
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Subscriber
  auto on_twist = std::bind(&DynamicRemover::on_twist, this, _1);
  auto cb = std::bind(&DynamicRemover::on_synchro, this, _1, _2);
  sub_twist_ = create_subscription<TwistStamped>("/localization/twist/kalman/twist", 10, on_twist);
  synchro_subscriber_.set_callback(std::move(cb));

  // Publisher
  pub_cur_image_ = create_publisher<Image>("reprojected_image", 10);
  pub_old_image_ = create_publisher<Image>("old_reprojected_image", 10);
  pub_cloud_ = create_publisher<PointCloud2>("filtered_lsd", 10);
}

void DynamicRemover::on_twist(TwistStamped::ConstSharedPtr msg) { twist_list_.push_back(msg); }

void DynamicRemover::try_define_param()
{
  if (param_.has_value()) return;

  std::optional<Sophus::SE3f> extrinsic = tf_subscriber_.se3f(info_.get_frame_id(), "base_link");
  if (!extrinsic.has_value()) return;
  if (info_.is_camera_info_nullopt()) return;

  const Eigen::Matrix3f K = info_.intrinsic();
  const Eigen::Matrix3f Kinv = K.inverse();

  param_ = Parameter{*extrinsic, K, Kinv};
  RCLCPP_INFO_STREAM(get_logger(), "extrinsic & intrinsic are ready");
}

DynamicRemover::ProjectFunc DynamicRemover::define_projection_function(const Sophus::SE3f & odom)
{
  return [this, odom](const cv::Point2f & src) -> std::optional<cv::Point2f> {
    const Eigen::Vector3f t = param_->extrinsic_.translation();
    const Eigen::Quaternionf q(param_->extrinsic_.unit_quaternion());
    const Eigen::Matrix3f K = param_->K_;
    const Eigen::Matrix3f Kinv = param_->Kinv_;

    Eigen::Vector3f u3(src.x, src.y, 1);
    Eigen::Vector3f u_bearing = (q * Kinv * u3).normalized();
    if (u_bearing.z() > -0.01) return std::nullopt;

    float u_distance = -t.z() / u_bearing.z();
    Eigen::Vector3f v;
    v.x() = t.x() + u_bearing.x() * u_distance;
    v.y() = t.y() + u_bearing.y() * u_distance;
    v.z() = 0;

    // NOTE:
    v = odom * v;

    Eigen::Vector3f from_camera = q.conjugate() * (v - t);
    if (from_camera.z() < 0.01) return std::nullopt;
    Eigen::Vector3f reprojected = K * from_camera / from_camera.z();
    return cv::Point2f{reprojected.x(), reprojected.y()};
  };
}

void DynamicRemover::on_synchro(const Image & image_msg, const PointCloud2 & lsd_msg)
{
  common::Timer timer;

  image_list_.push_back(image_msg);
  if (image_list_.size() < 2) return;
  const Image & old_image_msg = image_list_.front();

  try_define_param();
  if (!param_.has_value()) return;

  // Convert ROS messages to native messages
  pcl::PointCloud<pcl::PointNormal>::Ptr lsd{new pcl::PointCloud<pcl::PointNormal>()};
  pcl::fromROSMsg(lsd_msg, *lsd);
  cv::Mat old_image = common::decompress_to_cv_mat(old_image_msg);
  cv::Mat cur_image = common::decompress_to_cv_mat(image_msg);

  // Accumulate travel distance
  const rclcpp::Time old_stamp{old_image_msg.header.stamp};
  const rclcpp::Time cur_stamp{image_msg.header.stamp};
  Sophus::SE3f odom = accumulate_travel_distance(old_stamp, cur_stamp);
  RCLCPP_INFO_STREAM(get_logger(), "relative position: " << odom.translation().transpose());

  // Define projection function from relative pose
  ProjectFunc func = define_projection_function(odom);

  std::vector<TransformPair> pairs = make_transform_pairs(func, *lsd);
  RCLCPP_INFO_STREAM(get_logger(), "successfully transformed segments: " << pairs.size());

  // Search offsets to refine alignment
  std::unordered_map<size_t, GapResult> gap_map = compute_gap(pairs, old_image, cur_image);
  RCLCPP_INFO_STREAM(get_logger(), "successfully compute gap");

  // Visualize & publish
  visualize_and_publish(pairs, gap_map, old_image, cur_image);
  RCLCPP_INFO_STREAM(get_logger(), "publish");

  // Filt line segments using gap, then publish
  publish_cloud(*lsd, gap_map, cur_stamp);

  // Pop
  pop_obsolete_msg();

  RCLCPP_INFO_STREAM(get_logger(), "whole time: " << timer);
}

void DynamicRemover::publish_cloud(
  const pcl::PointCloud<pcl::PointNormal> & src, const std::unordered_map<size_t, GapResult> & gaps,
  const rclcpp::Time & stamp)
{
  pcl::PointCloud<pcl::PointNormal> dst;
  for (const auto & [id, gap] : gaps) {
    if (gap.gap < gap_threshold_) {
      dst.push_back(src.at(id));
    }
  }
  common::publish_cloud(*pub_cloud_, dst, stamp);
}

void DynamicRemover::visualize_and_publish(
  const std::vector<TransformPair> & pairs, const std::unordered_map<size_t, GapResult> & gap_map,
  const cv::Mat & old_image, const cv::Mat & cur_image)
{
  // Visualize old image using best offset
  cv::Mat draw_old_image = old_image.clone();
  cv::Rect rect(0, 0, old_image.cols, old_image.rows);
  for (const auto & pair : pairs) {
    const cv::Point2f offset = gap_map.at(pair.id).final_offset;
    for (const auto & dst : pair.dst) {
      if (rect.contains(dst)) draw_old_image.at<cv::Vec3b>(dst) = cv::Vec3b(255, 0, 0);
      if (rect.contains(dst + offset))
        draw_old_image.at<cv::Vec3b>(dst + offset) = cv::Vec3b(0, 255, 255);
    }
  }

  // Visualize cur image using gap score
  cv::Mat draw_cur_image = cur_image.clone();
  for (const auto & pair : pairs) {
    const float gap = gap_map.at(pair.id).gap;

    float score = (gap / gap_threshold_ * 255.f);
    cv::Vec3b color(0, 0, 255);
    if (score < 255) color = cv::Vec3b(score, score, score);

    for (const auto & src : pair.src) {
      if (rect.contains(src)) draw_cur_image.at<cv::Vec3b>(src) = color;
    }
  }
  common::publish_image(*pub_old_image_, draw_old_image, get_clock()->now());
  common::publish_image(*pub_cur_image_, draw_cur_image, get_clock()->now());
}

std::unordered_map<size_t, DynamicRemover::GapResult> DynamicRemover::compute_gap(
  const std::vector<TransformPair> & pairs, const cv::Mat & old_image, const cv::Mat & cur_image)
{
  std::vector<cv::Point2f> offset_bases = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 0},
                                           {0, 1},   {1, -1}, {1, 0},  {1, 1}};

  const cv::Rect rect(0, 0, old_image.cols, old_image.rows);

  // TODO: someday, compute both x&y gradient and compute inner product
  cv::Mat old_edge, cur_edge;
  cv::cvtColor(old_image, old_edge, cv::COLOR_BGR2GRAY);
  cv::cvtColor(cur_image, cur_edge, cv::COLOR_BGR2GRAY);
  cv::Sobel(cur_edge, cur_edge, CV_8U, 1, 0);
  cv::Sobel(old_edge, old_edge, CV_8U, 1, 0);

  auto lambda_compute_gap = [&](const TransformPair & pair, const cv::Point2f & offset) -> float {
    int gap = 0;
    int gap_cnt = 0;

    for (size_t i = 0; i < pair.src.size(); ++i) {
      if (rect.contains(pair.src.at(i)) && rect.contains(pair.dst.at(i) + offset)) {
        // TODO:
        // cv::Vec3b s = cur_image.at<cv::Vec3b>(pair.src.at(i));
        // cv::Vec3b d = old_image.at<cv::Vec3b>(pair.dst.at(i) + offset);
        // gap += (s - d).dot(s - d);
        unsigned char s = cur_edge.at<unsigned char>(pair.src.at(i));
        unsigned char d = old_edge.at<unsigned char>(pair.dst.at(i) + offset);
        gap += (s - d) * (s - d);
        gap_cnt++;
      }
    }
    return gap / static_cast<float>(gap_cnt);
  };

  std::unordered_map<size_t, GapResult> gap_results;

  for (const auto & pair : pairs) {
    // First search
    float best_gap = std::numeric_limits<float>::max();
    cv::Point2f best_offset(0, 0);

    for (int itr = 0; itr < search_iteration_max_; itr++) {
      float tmp_best_gap = std::numeric_limits<float>::max();
      cv::Point2f tmp_best_offset(0, 0);

      for (const auto & offset : offset_bases) {
        int gap = lambda_compute_gap(pair, offset + best_offset);
        if (gap < tmp_best_gap) {
          tmp_best_gap = gap;
          tmp_best_offset = offset;
        }
      }

      best_gap = tmp_best_gap;
      best_offset = best_offset + tmp_best_offset;
    }

    GapResult result;
    result.id = pair.id;
    result.final_offset = best_offset;
    result.gap = best_gap;
    gap_results.emplace(pair.id, result);
  }

  return gap_results;
}

std::vector<DynamicRemover::TransformPair> DynamicRemover::make_transform_pairs(
  ProjectFunc func, pcl::PointCloud<pcl::PointNormal> & segments)
{
  std::vector<TransformPair> pairs;

  for (size_t i = 0; i < segments.size(); ++i) {
    const auto & ls = segments.at(i);
    TransformPair pair;
    pair.id = i;

    std::vector<cv::Point2i> polygon = line_to_polygon({ls.x, ls.y}, {ls.normal_x, ls.normal_y});
    for (const auto & p : polygon) {
      std::optional<cv::Point2f> opt = func(p);

      if (opt.has_value()) {
        pair.src.push_back(p);
        pair.dst.push_back(*opt);
      }
    }

    // If at least one pixel are transformed
    if (!pair.src.empty()) {
      pairs.push_back(pair);
    }
  }

  return pairs;
}

std::vector<cv::Point2i> DynamicRemover::line_to_polygon(
  const cv::Point2f & from, const cv::Point2f & to) const
{
  cv::Point2f upper_left, bottom_right;
  upper_left.x = std::min(from.x, to.x) - 2;
  upper_left.y = std::min(from.y, to.y) - 2;
  bottom_right.x = std::max(from.x, to.x) + 2;
  bottom_right.y = std::max(from.y, to.y) + 2;

  cv::Size size(bottom_right.x - upper_left.x, bottom_right.y - upper_left.y);
  cv::Mat canvas = cv::Mat::zeros(size, CV_8UC1);

  cv::line(
    canvas, from - upper_left, to - upper_left, cv::Scalar::all(255), polygon_thick_, cv::LINE_8);

  std::vector<cv::Point2i> non_zero;
  cv::findNonZero(canvas, non_zero);
  for (cv::Point2i & p : non_zero) {
    p += cv::Point2i(upper_left);
  }
  return non_zero;
}

Sophus::SE3f DynamicRemover::accumulate_travel_distance(
  const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp) const
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

void DynamicRemover::pop_obsolete_msg()
{
  if (image_list_.size() < backward_frame_interval_) {
    return;
  }

  rclcpp::Time oldest_stamp = image_list_.front().header.stamp;
  image_list_.pop_front();

  for (auto itr = twist_list_.begin(); itr != twist_list_.end();) {
    if (rclcpp::Time((*itr)->header.stamp) > oldest_stamp) break;
    itr = twist_list_.erase(itr);
  }
}

}  // namespace pcdless::dynamic_remover
