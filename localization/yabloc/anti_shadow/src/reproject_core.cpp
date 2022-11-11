#include "anti_shadow/reproject.hpp"

#include <vml_common/cv_decompress.hpp>
#include <vml_common/pub_sub.hpp>
#include <vml_common/timer.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace imgproc
{
Reprojector::Reprojector()
: Node("reprojector"),
  info_(this),
  tf_subscriber_(this->get_clock()),
  synchro_subscriber_(this, "/sensing/camera/undistorted/image_raw", "lsd_cloud"),
  min_segment_length_(declare_parameter<float>("min_segment_length", 0.5)),
  gain_(declare_parameter<float>("gain", 0.1))
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Subscriber
  auto on_twist = std::bind(&Reprojector::onTwist, this, _1);
  sub_twist_ =
    create_subscription<TwistStamped>("/localization/trajectory/kalman/twist", 10, on_twist);
  auto cb = std::bind(&Reprojector::onSynchro, this, _1, _2);
  synchro_subscriber_.setCallback(std::move(cb));

  // Publisher
  pub_image_ = create_publisher<Image>("reprojected_image", 10);
}

void Reprojector::onTwist(TwistStamped::ConstSharedPtr msg) { twist_list_.push_back(msg); }

void Reprojector::tryDefineParam()
{
  if (param_.has_value()) return;

  std::optional<Sophus::SE3f> extrinsic = tf_subscriber_.se3f(info_.getFrameId(), "base_link");
  if (!extrinsic.has_value()) return;
  if (info_.isCameraInfoNullOpt()) return;

  const Eigen::Matrix3f K = info_.intrinsic();
  const Eigen::Matrix3f Kinv = K.inverse();

  param_ = Parameter{*extrinsic, K, Kinv};
  RCLCPP_INFO_STREAM(get_logger(), "extrinsic & intrinsic are ready");
}

Reprojector::ProjectFunc Reprojector::defineProjectionFunction(const Sophus::SE3f & odom)
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

void Reprojector::onSynchro(const Image & image_msg, const PointCloud2 & lsd_msg)
{
  image_list_.push_back(image_msg);
  if (image_list_.size() < 2) return;
  const Image & old_image_msg = image_list_.front();

  tryDefineParam();
  if (!param_.has_value()) return;

  // Convert ROS messages to native messages
  pcl::PointCloud<pcl::PointNormal>::Ptr lsd{new pcl::PointCloud<pcl::PointNormal>()};
  pcl::fromROSMsg(lsd_msg, *lsd);
  cv::Mat old_image = vml_common::decompress2CvMat(old_image_msg);
  cv::Mat cur_image = vml_common::decompress2CvMat(image_msg);

  // Accumulate travel distance
  const rclcpp::Time old_stamp{old_image_msg.header.stamp};
  const rclcpp::Time cur_stamp{image_msg.header.stamp};
  Sophus::SE3f odom = accumulateTravelDistance(old_stamp, cur_stamp);
  RCLCPP_INFO_STREAM(get_logger(), "relative possition: " << odom.translation().transpose());

  // Define projection function from relative pose
  ProjectFunc func = defineProjectionFunction(odom);

  std::vector<TransformPair> pairs = makeTransformPairs(func, *lsd);
  std::cout << "pairs " << pairs.size() << std::endl;

  cv::Mat draw_image = drawTransformedPixel(pairs, old_image, cur_image);
  vml_common::publishImage(*pub_image_, draw_image, cur_stamp);
  std::cout << "draw & publish" << std::endl;

  popObsoleteMsg();
}

cv::Mat Reprojector::drawTransformedPixel(
  const std::vector<TransformPair> & pairs, const cv::Mat & old_image, const cv::Mat & cur_image)
{
  std::vector<cv::Point2i> offsets = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 0},
                                      {0, 1},   {1, -1}, {1, 0},  {1, 1}};

  auto compute_gap = [&](const TransformPair & pair, const cv::Point2f & offset) -> int {
    int gap = 0;
    const int N = pair.src.size();
    // TODO: check out of image
    for (int i = 0; i < N; ++i) {
      cv::Vec3b s = cur_image.at<cv::Vec3b>(pair.src.at(i));
      cv::Vec3b d = old_image.at<cv::Vec3b>(pair.dst.at(i) + offset);
      gap += (s - d).dot(s - d);
    }
    return gap;
  };

  cv::Mat draw_image = old_image.clone();

  for (const auto & pair : pairs) {
    int best_gap = std::numeric_limits<int>::max();
    cv::Point2f best_offset;
    for (const auto & offset : offsets) {
      int gap = compute_gap(pair, offset);
      if (gap < best_gap) {
        best_gap = gap;
        best_offset = offset;
      }
    }

    // Draw
    const int N = pair.src.size();
    for (int i = 0; i < N; ++i) {
      const auto & d = pair.dst.at(i);
      draw_image.at<cv::Vec3b>(d + best_offset) = cv::Vec3b(0, 255, 255);
    }
  }

  return draw_image;
}

std::vector<Reprojector::TransformPair> Reprojector::makeTransformPairs(
  ProjectFunc func, pcl::PointCloud<pcl::PointNormal> & segments)
{
  std::vector<TransformPair> pairs;

  for (const auto & ls : segments) {
    TransformPair pair;
    std::vector<cv::Point2i> polygon = line2Polygon({ls.x, ls.y}, {ls.normal_x, ls.normal_y});

    for (const auto & p : polygon) {
      std::optional<cv::Point2f> opt = func(p);

      if (opt.has_value()) {
        pair.src.push_back(p);
        pair.dst.push_back(*opt);
      }
    }
    if (!pair.src.empty()) pairs.push_back(pair);
  }

  return pairs;
}

std::vector<cv::Point2i> Reprojector::line2Polygon(
  const cv::Point2f & from, const cv::Point2f & to) const
{
  cv::Point2f upper_left, bottom_right;
  upper_left.x = std::min(from.x, to.x) - 2;
  upper_left.y = std::min(from.y, to.y) - 2;
  bottom_right.x = std::max(from.x, to.x) + 2;
  bottom_right.y = std::max(from.y, to.y) + 2;

  cv::Size size(bottom_right.x - upper_left.x, bottom_right.y - upper_left.y);
  cv::Mat canvas = cv::Mat::zeros(size, CV_8UC1);

  cv::line(canvas, from - upper_left, to - upper_left, cv::Scalar::all(255), 2, cv::LINE_8);

  std::vector<cv::Point2i> non_zero;
  cv::findNonZero(canvas, non_zero);
  for (cv::Point2i & p : non_zero) {
    p += cv::Point2i(upper_left);
  }
  return non_zero;
}

Sophus::SE3f Reprojector::accumulateTravelDistance(
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

void Reprojector::popObsoleteMsg()
{
  if (image_list_.size() < 5) {
    return;
  }

  rclcpp::Time oldest_stamp = image_list_.front().header.stamp;
  image_list_.pop_front();

  for (auto itr = twist_list_.begin(); itr != twist_list_.end();) {
    if (rclcpp::Time((*itr)->header.stamp) > oldest_stamp) break;
    itr = twist_list_.erase(itr);
  }
}

}  // namespace imgproc
