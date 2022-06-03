#include "common/util.hpp"
#include "sign_detector/ll2_util.hpp"
#include "sign_detector/mask_lsd.hpp"

#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <sophus/geometry.hpp>

#include <cv_bridge/cv_bridge.h>
#include <lanelet2_core/LaneletMap.h>

void MaskLsd::synchroCallback(
  const geometry_msgs::msg::PoseStamped & pose,
  const sensor_msgs::msg::CompressedImage & compressed_image)
{
  auto dt = rclcpp::Time(pose.header.stamp) - rclcpp::Time(compressed_image.header.stamp);
  std::cout << "dt: " << dt.seconds() << std::endl;

  cv::Mat image = decompress2CvMat(compressed_image);
  cv::Size size = image.size();

  if (!info_.has_value()) return;
  cv::Mat K = cv::Mat(cv::Size(3, 3), CV_64FC1, (void *)(info_->k.data()));
  cv::Mat D = cv::Mat(cv::Size(5, 1), CV_64FC1, (void *)(info_->d.data()));
  cv::Mat undistorted;
  cv::undistort(image, undistorted, K, D, K);
  image = undistorted;

  const int WIDTH = 800;
  const float SCALE = 1.0f * WIDTH / size.width;
  const int HEIGHT = SCALE * size.height;
  cv::resize(image, image, cv::Size(WIDTH, HEIGHT));
  cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

  scaled_intrinsic = SCALE * K;
  scaled_intrinsic->at<double>(2, 2) = 1;

  // LSD
  cv::Mat lines;
  lsd->detect(image, lines);
  lsd->drawSegments(image, lines);

  // LL2
  pcl::PointCloud<pcl::PointNormal> near_linestring = poseCallback(pose);
  std::cout << "ll2 lines" << near_linestring.size() << std::endl;

  // Overlay LL2 on LSD image
  overlayLl2(image, near_linestring, pose.pose);

  cv::imshow("lsd", image);
  cv::waitKey(1);
}

void MaskLsd::overlayLl2(
  cv::Mat & lsd_image, const pcl::PointCloud<pcl::PointNormal> & ll2,
  const geometry_msgs::msg::Pose & pose)
{
  // const auto ori = pose.orientation;
  // const auto pos = pose.position;
  // Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
  // Eigen::Vector3f t(pos.x, pos.y, pos.z);
  Eigen::Vector3f t = camera_extrinsic_->translation();
  Eigen::Matrix3f R = camera_extrinsic_->rotation();

  Eigen::Matrix3f K;
  cv::cv2eigen(scaled_intrinsic.value(), K);

  cv::Mat ll2_image = cv::Mat::zeros(lsd_image.size(), CV_8UC3);
  for (const pcl::PointNormal & pn : ll2) {
    Eigen::Vector3f p1 = pn.getVector3fMap();
    Eigen::Vector3f p2 = pn.getNormalVector3fMap();
    Eigen::Vector3f u = K * R.transpose() * (p1 - t);
    Eigen::Vector3f v = K * R.transpose() * (p2 - t);

    if (u.z() < 1e-3f) continue;
    if (v.z() < 1e-3f) continue;
    u = u / u.z();
    v = v / v.z();
    cv::line(
      ll2_image, cv::Point2f(u.x(), u.y()), cv::Point2f(v.x(), v.y()), cv::Scalar(0, 255, 255), 5);
  }

  cv::addWeighted(lsd_image, 0.8f, ll2_image, 0.4f, 1, lsd_image);
}

void MaskLsd::listenExtrinsicTf(const std::string & frame_id)
{
  try {
    geometry_msgs::msg::TransformStamped ts =
      tf_buffer_->lookupTransform("base_link", frame_id, tf2::TimePointZero);
    Eigen::Vector3f p;
    p.x() = ts.transform.translation.x;
    p.y() = ts.transform.translation.y;
    p.z() = ts.transform.translation.z;

    Eigen::Quaternionf q;
    q.w() = ts.transform.rotation.w;
    q.x() = ts.transform.rotation.x;
    q.y() = ts.transform.rotation.y;
    q.z() = ts.transform.rotation.z;

    camera_extrinsic_ = Eigen::Affine3f::Identity();
    camera_extrinsic_->translation() = p + gnss_to_baselink_;
    camera_extrinsic_->matrix().topLeftCorner(3, 3) = q.toRotationMatrix();
  } catch (tf2::TransformException & ex) {
  }
}

pcl::PointCloud<pcl::PointNormal> MaskLsd::poseCallback(
  const geometry_msgs::msg::PoseStamped & pose_stamped)
{
  pcl::PointCloud<pcl::PointNormal> near_linestring;
  if (linestrings_ == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 500, "wating for /map/vector_map");
    return near_linestring;
  }
  Eigen::Vector3f pose;
  {
    const auto p = pose_stamped.pose.position;
    pose << p.x, p.y, p.z;
  }

  // Compute distance between pose and linesegment of linestring
  auto checkIntersection = [this, pose](const pcl::PointNormal & pn) -> bool {
    const Eigen::Vector3f from = pn.getVector3fMap() - pose;
    const Eigen::Vector3f to = pn.getNormalVector3fMap() - pose;
    Eigen::Vector3f dir = to - from;
    float inner = from.dot(dir);
    if (std::abs(inner) < 1e-3f) {
      return from.norm() < 1.42 * this->max_range_;
    }

    float mu = std::clamp(dir.squaredNorm() / inner, 0.f, 1.0f);
    Eigen::Vector3f nearest = from + dir * mu;
    return nearest.norm() < 2 * 1.42 * this->max_range_;
  };

  float w = pose_stamped.pose.orientation.w;
  float z = pose_stamped.pose.orientation.z;
  Eigen::Matrix3f rotation =
    Eigen::AngleAxisf(-2.f * std::atan2(z, w), Eigen::Vector3f::UnitZ()).matrix();

  for (const pcl::PointNormal & pn : *linestrings_) {
    if (checkIntersection(pn)) {
      pcl::PointNormal relative_pn;
      relative_pn.getVector3fMap() = rotation * (pn.getVector3fMap() - pose);
      relative_pn.getNormalVector3fMap() = rotation * (pn.getNormalVector3fMap() - pose);
      relative_pn.z = 0;
      relative_pn.normal_z = 0;
      near_linestring.push_back(relative_pn);
    }
  }
  return near_linestring;
}

void MaskLsd::mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map = fromBinMsg(msg);
  linestrings_ = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

  for (lanelet::LineString3d & line : viz_lanelet_map->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (
      attr.value() != "zebra_marking" && attr.value() != "virtual" && attr.value() != "line_thin" &&
      attr.value() != "pedestrian_marking" && attr.value() != "stop_line")
      continue;

    bool is_dual = (attr.value() == "line_thin");

    std::optional<lanelet::ConstPoint3d> from = std::nullopt;
    for (const lanelet::ConstPoint3d p : line) {
      if (from.has_value()) {
        pcl::PointNormal pn;
        pn.x = from->x();
        pn.y = from->y();
        pn.z = 0;
        pn.normal_x = p.x();
        pn.normal_y = p.y();
        pn.normal_z = 0;
        // if(is_dual){ TODO: }
        linestrings_->push_back(pn);
      }
      from = p;
    }
  }
  std::cout << "map: " << linestrings_->size() << std::endl;
}