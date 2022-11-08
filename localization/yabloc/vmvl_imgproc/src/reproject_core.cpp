#include "vmvl_imgproc/reproject.hpp"

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
  min_segment_length_(declare_parameter<float>("min_segment_length", 0.5))
{
  using std::placeholders::_1;

  // Subscriber
  auto on_image = std::bind(&Reprojector::onImage, this, _1);
  auto on_lsd = std::bind(&Reprojector::onLineSegments, this, _1);
  auto on_twist = std::bind(&Reprojector::onTwist, this, _1);
  sub_image_ = create_subscription<Image>("/sensing/camera/undistorted/image_raw", 10, on_image);
  sub_lsd_ = create_subscription<PointCloud2>("lsd_cloud", 10, on_lsd);
  sub_twist_ =
    create_subscription<TwistStamped>("/localization/trajectory/kalman/twist", 10, on_twist);

  // Publisher
  pub_image_ = create_publisher<Image>("reprojected_image", 10);
}

void Reprojector::onTwist(TwistStamped::ConstSharedPtr msg) { twist_list_.push_back(msg); }

void Reprojector::onImage(Image::ConstSharedPtr msg) { image_list_.push_back(msg); }

void Reprojector::onLineSegments(const PointCloud2 & msg)
{
  if (!image_list_.empty()) {
    reproject(*image_list_.front(), msg);
  }
  popObsoleteMsg();
}

void Reprojector::reproject(const Image & old_image_msg, const PointCloud2 & cloud_msg)
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

  std::cout << "extrinsic & intrinsic are ready" << std::endl;

  cv::Mat old_image = vml_common::decompress2CvMat(old_image_msg);
  rclcpp::Time old_stamp{old_image_msg.header.stamp};
  rclcpp::Time cur_stamp{cloud_msg.header.stamp};

  // Compute travel distance
  Sophus::SE3f pose = accumulateTravelDistance(old_stamp, cur_stamp);
  std::cout << "relative pose: " << pose.translation().transpose() << ", "
            << pose.unit_quaternion().coeffs().transpose() << std::endl;

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
  auto reproject_func = [K, q, t](const Eigen::Vector3f & u) -> std::optional<Eigen::Vector3f> {
    Eigen::Vector3f from_camera = q.conjugate() * (u - t);
    if (from_camera.z() < 0.01) return std::nullopt;
    Eigen::Vector3f reprojected = K * from_camera / from_camera.z();
    return reprojected;
  };
  auto cv_pt2 = [](const Eigen::Vector3f & v) -> cv::Point { return {v.x(), v.y()}; };

  // Reproject linesegments
  int draw_cnt = 0;
  for (const auto & ls : *lsd) {
    // project segment on ground
    std::optional<Eigen::Vector3f> opt1 = project_func(ls.getVector3fMap());
    std::optional<Eigen::Vector3f> opt2 = project_func(ls.getNormalVector3fMap());
    if (!opt1.has_value()) continue;
    if (!opt2.has_value()) continue;
    float length = (opt1.value() - opt2.value()).norm();
    if (length < min_segment_length_) continue;

    // transform segment on ground
    Eigen::Vector3f transformed1 = pose * opt1.value();
    Eigen::Vector3f transformed2 = pose * opt2.value();

    // reproject segment from ground
    std::optional<Eigen::Vector3f> re_opt1 = reproject_func(transformed1);
    std::optional<Eigen::Vector3f> re_opt2 = reproject_func(transformed2);
    if (!re_opt1.has_value()) continue;
    if (!re_opt2.has_value()) continue;

    cv::line(old_image, cv_pt2(*re_opt1), cv_pt2(*re_opt2), cv::Scalar(0, 0, 255), 2);
    draw_cnt++;
  }

  std::cout << "finish reproject()  " << draw_cnt << std::endl;
  vml_common::publishImage(*pub_image_, old_image, cloud_msg.header.stamp);
}

Sophus::SE3f Reprojector::accumulateTravelDistance(
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

void Reprojector::popObsoleteMsg()
{
  if (image_list_.size() < 5) {
    return;
  }

  rclcpp::Time oldest_stamp = image_list_.front()->header.stamp;
  image_list_.pop_front();

  for (auto itr = twist_list_.begin(); itr != twist_list_.end();) {
    if (rclcpp::Time((*itr)->header.stamp) > oldest_stamp) break;
    itr = twist_list_.erase(itr);
  }
}

}  // namespace imgproc
