#include "anti_shadow/mapping.hpp"

#include <vml_common/cv_decompress.hpp>
#include <vml_common/pub_sub.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace imgproc
{
Mapping::Mapping()
: Node("reprojector"),
  info_(this),
  tf_subscriber_(this->get_clock()),
  min_segment_length_(declare_parameter<float>("min_segment_length", 0.5))
{
  using std::placeholders::_1;

  // Subscriber
  auto on_lsd = std::bind(&Mapping::onLineSegments, this, _1);
  auto on_twist = std::bind(&Mapping::onTwist, this, _1);
  sub_lsd_ = create_subscription<PointCloud2>("lsd_cloud", 10, on_lsd);
  sub_twist_ =
    create_subscription<TwistStamped>("/localization/trajectory/kalman/twist", 10, on_twist);

  // Publisher
  pub_image_ = create_publisher<Image>("mapping_image", 10);

  histogram_image_ = 128 * cv::Mat::ones(cv::Size(2 * IMAGE_RADIUS, 2 * IMAGE_RADIUS), CV_8UC1);
}

void Mapping::onTwist(TwistStamped::ConstSharedPtr msg) { twist_list_.push_back(msg); }

cv::Point2f Mapping::cv_pt2(const Eigen::Vector3f & v) const
{
  return {-v.y() / METRIC_PER_PIXEL + IMAGE_RADIUS, -v.x() / METRIC_PER_PIXEL + IMAGE_RADIUS};
}

Eigen::Vector3f Mapping::eigen_vec3f(const cv::Point2f & p) const
{
  return {-(p.y - IMAGE_RADIUS) * METRIC_PER_PIXEL, -(p.x - IMAGE_RADIUS) * METRIC_PER_PIXEL, 0};
}

void Mapping::onLineSegments(const PointCloud2 & msg)
{
  static std::optional<rclcpp::Time> last_stamp{std::nullopt};
  if (last_stamp.has_value()) {
    transformImage(*last_stamp, msg.header.stamp);
  }
  last_stamp = msg.header.stamp;

  draw(msg);
  popObsoleteMsg(*last_stamp);

  cv::Mat color_image;
  cv::applyColorMap(histogram_image_, color_image, cv::COLORMAP_JET);
  vml_common::publishImage(*pub_image_, color_image, msg.header.stamp);
}

void Mapping::transformImage(const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp)
{
  // Transform histogram image
  Sophus::SE3f odom = accumulateTravelDistance(from_stamp, to_stamp).inverse();

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
    cv::Scalar::all(128));
}

void Mapping::draw(const PointCloud2 & cloud_msg)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr lsd{new pcl::PointCloud<pcl::PointNormal>()};
  pcl::fromROSMsg(cloud_msg, *lsd);
  RCLCPP_INFO_STREAM(get_logger(), "segments size:" << lsd->size());

  // Check intrinsic & extrinsic
  std::optional<Eigen::Affine3f> camera_extrinsic = tf_subscriber_(info_.getFrameId(), "base_link");
  if (!camera_extrinsic.has_value()) return;
  if (info_.isCameraInfoNullOpt()) return;
  const Eigen::Matrix3f K = info_.intrinsic();
  const Eigen::Matrix3f Kinv = K.inverse();

  const Eigen::Vector3f t = camera_extrinsic->translation();
  const Eigen::Quaternionf q(camera_extrinsic->rotation());
  auto project_func = [Kinv, q, t](const Eigen::Vector3f & u) -> std::optional<Eigen::Vector3f> {
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

  // Project segment on ground
  cv::Mat increment_image = cv::Mat::zeros(histogram_image_.size(), CV_8UC1);
  for (const auto & ls : *lsd) {
    std::optional<Eigen::Vector3f> opt1 = project_func(ls.getVector3fMap());
    std::optional<Eigen::Vector3f> opt2 = project_func(ls.getNormalVector3fMap());
    if (!opt1.has_value()) continue;
    if (!opt2.has_value()) continue;
    float length = (opt1.value() - opt2.value()).norm();
    if (length < min_segment_length_) continue;

    cv::line(increment_image, cv_pt2(*opt1), cv_pt2(*opt2), cv::Scalar::all(255), 1);
  }
  histogram_image_ += increment_image;

  // Decrease beflief in unsee area
  {
    std::vector<cv::Point2i> points;
    auto project = [this, &points, project_func](const Eigen::Vector3f & u) -> void {
      std::optional<Eigen::Vector3f> p = project_func(u);
      points.push_back(cv_pt2(*p));
    };
    project({0, 270, 1});
    project({800, 270, 1});
    project({800, 516, 1});
    project({0, 516, 1});

    cv::Mat decrease_image = cv::Mat::zeros(histogram_image_.size(), CV_8UC1);
    cv::fillPoly(decrease_image, points, cv::Scalar::all(10), 8, 0);
    histogram_image_ -= decrease_image;
  }
}

Sophus::SE3f Mapping::accumulateTravelDistance(
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

void Mapping::popObsoleteMsg(const rclcpp::Time & oldest_stamp)
{
  for (auto itr = twist_list_.begin(); itr != twist_list_.end();) {
    if (rclcpp::Time((*itr)->header.stamp) > oldest_stamp) break;
    itr = twist_list_.erase(itr);
  }
}

}  // namespace imgproc
