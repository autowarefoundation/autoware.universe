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
  min_segment_length_(declare_parameter<float>("min_segment_length", 0.5))
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

void Reprojector::tryDefineProjectFunction()
{
  if (project_func && reproject_func) {
    return;
  }

  // Check intrinsic & extrinsic
  std::optional<Eigen::Affine3f> camera_extrinsic = tf_subscriber_(info_.getFrameId(), "base_link");
  if (!camera_extrinsic.has_value()) return;
  if (info_.isCameraInfoNullOpt()) return;
  const Eigen::Matrix3f K = info_.intrinsic();
  const Eigen::Matrix3f Kinv = K.inverse();

  RCLCPP_INFO_STREAM(get_logger(), "extrinsic & intrinsic are ready");

  // Define projection function
  const Eigen::Vector3f t = camera_extrinsic->translation();
  const Eigen::Quaternionf q(camera_extrinsic->rotation());

  project_func = [Kinv, q, t](const Eigen::Vector3f & u) -> std::optional<Eigen::Vector3f> {
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

  reproject_func = [K, q, t](const Eigen::Vector3f & u) -> std::optional<Eigen::Vector3f> {
    Eigen::Vector3f from_camera = q.conjugate() * (u - t);
    if (from_camera.z() < 0.01) return std::nullopt;
    Eigen::Vector3f reprojected = K * from_camera / from_camera.z();
    return reprojected;
  };
}

void Reprojector::onSynchro(const Image & image_msg, const PointCloud2 & lsd_msg)
{
  image_list_.push_back(image_msg);

  tryDefineProjectFunction();

  if (image_list_.size() > 2) {
    reproject(image_list_.front(), image_msg, lsd_msg);
  }

  // {
  //   cv::Mat image = vml_common::decompress2CvMat(image_msg);
  //   pcl::PointCloud<pcl::PointNormal>::Ptr lsd{new pcl::PointCloud<pcl::PointNormal>()};
  //   pcl::fromROSMsg(lsd_msg, *lsd);
  //   for (const auto l : *lsd) {
  //     std::vector<cv::Point2i> polygon = line2Polygon({l.x, l.y}, {l.normal_x, l.normal_y});
  //     for (const auto & p : polygon) {
  //       image.at<cv::Vec3b>(p) = cv::Vec3b(0, 255, 255);
  //     }
  //   }

  //   vml_common::publishImage(*pub_image_, image, image_msg.header.stamp);
  // }

  popObsoleteMsg();
}

std::vector<cv::Point2i> Reprojector::line2Polygon(const cv::Point2f & from, const cv::Point2f & to)
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

void Reprojector::reproject(
  const Image & old_image_msg, const Image & current_image_msg, const PointCloud2 & lsd_msg)
{
  if ((!project_func) || (!reproject_func)) return;

  // Convert ROS messages to native messages
  pcl::PointCloud<pcl::PointNormal>::Ptr lsd{new pcl::PointCloud<pcl::PointNormal>()};
  pcl::fromROSMsg(lsd_msg, *lsd);
  cv::Mat old_image = vml_common::decompress2CvMat(old_image_msg);
  cv::Mat current_image = vml_common::decompress2CvMat(current_image_msg);

  rclcpp::Time old_stamp{old_image_msg.header.stamp};
  rclcpp::Time cur_stamp{current_image_msg.header.stamp};

  // Accumulate travel distance
  Sophus::SE3f pose = accumulateTravelDistance(old_stamp, cur_stamp);
  RCLCPP_INFO_STREAM(get_logger(), "relative pose: " << pose.translation().transpose());
  RCLCPP_INFO_STREAM(
    get_logger(), "relative pose: " << pose.unit_quaternion().coeffs().transpose());

  auto cv_pt2 = [](const Eigen::Vector3f & v) -> cv::Point { return {v.x(), v.y()}; };

  // Reproject linesegments
  int draw_cnt = 0;
  for (const auto & ls : *lsd) {
    std::vector<cv::Point2i> polygon = line2Polygon({ls.x, ls.y}, {ls.normal_x, ls.normal_y});

    for (const auto & p : polygon) {
      // project segment on ground
      std::optional<Eigen::Vector3f> opt = project_func({p.x, p.y, 0});
      if (!opt.has_value()) continue;

      // transform segment on ground
      Eigen::Vector3f transformed = pose * opt.value();

      // reproject segment from ground
      std::optional<Eigen::Vector3f> re_opt = reproject_func(transformed);
      if (!re_opt.has_value()) continue;

      cv::Point2i src = p;
      cv::Point2i dst(re_opt->x(), re_opt->y());
      old_image.at<cv::Vec3b>(dst) = current_image.at<cv::Vec3b>(src);
      draw_cnt++;
    }
  }

  std::cout << "finish reproject()  " << draw_cnt << std::endl;
  vml_common::publishImage(*pub_image_, old_image, cur_stamp);
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

  rclcpp::Time oldest_stamp = image_list_.front().header.stamp;
  image_list_.pop_front();

  for (auto itr = twist_list_.begin(); itr != twist_list_.end();) {
    if (rclcpp::Time((*itr)->header.stamp) > oldest_stamp) break;
    itr = twist_list_.erase(itr);
  }
}

// cv::Mat Reprojector::applyPerspective(const cv::Mat & image)
// {
//   std::vector<cv::Point2f> src_points;
//   src_points.push_back(cv::Point2f(400, 450));
//   src_points.push_back(cv::Point2f(300, 400));
//   src_points.push_back(cv::Point2f(500, 450));
//   src_points.push_back(cv::Point2f(400, 350));
//   std::vector<cv::Point2f> dst_points;
//   for (int i = 0; i < 4; ++i) dst_points.push_back(project_func(src_points[i]));
//   cv::Mat warp_mat = cv::getPerspectiveTransform(src_points, dst_points);
//   cv::Mat warp_image;
//   cv::warpPerspective(image, warp_image, warp_mat, image.size());
//   return warp_image;
// }

}  // namespace imgproc
