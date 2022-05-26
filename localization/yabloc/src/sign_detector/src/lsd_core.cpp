#include "sign_detector/lsd.hpp"
#include "sign_detector/util.hpp"

#include <pcl_conversions/pcl_conversions.h>

void LineSegmentDetector::imageCallback(const sensor_msgs::msg::CompressedImage & msg) const
{
  sensor_msgs::msg::Image::ConstSharedPtr image_ptr = decompressImage(msg);
  cv::Mat image = cv_bridge::toCvCopy(*image_ptr, "rgb8")->image;
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

  std::chrono::time_point start = std::chrono::system_clock::now();
  cv::Mat lines;
  lsd->detect(image, lines);
  lsd->drawSegments(image, lines);

  auto dur = std::chrono::system_clock::now() - start;
  long ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
  RCLCPP_INFO_STREAM(this->get_logger(), cv::Size(WIDTH, HEIGHT) << " " << ms);

  {
    cv_bridge::CvImage raw_image;
    raw_image.header.stamp = msg.header.stamp;
    raw_image.header.frame_id = "map";
    raw_image.encoding = "bgr8";
    raw_image.image = image;
    pub_image_lsd_->publish(*raw_image.toImageMsg());
  }

  cv::Mat scaled_K = SCALE * K;
  scaled_K.at<double>(2, 2) = 1;
  projectEdgeOnPlane(lines, scaled_K, msg.header.stamp);
}

void LineSegmentDetector::listenExtrinsicTf(const std::string & frame_id)
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
    camera_extrinsic_->translation() = p;
    camera_extrinsic_->matrix().topLeftCorner(3, 3) = q.toRotationMatrix();
  } catch (tf2::TransformException & ex) {
  }
}

void LineSegmentDetector::projectEdgeOnPlane(
  const cv::Mat & lines, const cv::Mat & K_cv, const rclcpp::Time & stamp) const
{
  if (!camera_extrinsic_.has_value()) {
    RCLCPP_WARN_STREAM(this->get_logger(), "camera_extrinsic_ has not been initialized");
    return;
  }
  Eigen::Vector3f t = camera_extrinsic_->translation();
  Eigen::Quaternionf q(camera_extrinsic_->rotation());
  RCLCPP_INFO_STREAM(
    this->get_logger(), "transform: " << t.transpose() << " " << q.coeffs().transpose());

  // Convert to projected coordinate
  Eigen::Matrix3f K, K_inv;
  cv::cv2eigen(K_cv, K);
  K_inv = K.inverse();

  auto conv = [t, q, K_inv](
                const Eigen::Vector2f & u,
                const Eigen::Vector2f & v) -> std::optional<pcl::PointNormal> {
    Eigen::Vector3f u3(u.x(), u.y(), 1);
    Eigen::Vector3f v3(v.x(), v.y(), 1);
    Eigen::Vector3f u_bearing = (q * K_inv * u3).normalized();
    Eigen::Vector3f v_bearing = (q * K_inv * v3).normalized();
    if (u_bearing.z() > -0.02) return std::nullopt;
    if (v_bearing.z() > -0.02) return std::nullopt;
    float u_distance = -t.z() / u_bearing.z();
    float v_distance = -t.z() / v_bearing.z();
    pcl::PointNormal pn;
    pn.x = t.x() + u_bearing.x() * u_distance;
    pn.y = t.y() + u_bearing.y() * u_distance;
    pn.normal_x = t.x() + v_bearing.x() * v_distance;
    pn.normal_y = t.y() + v_bearing.y() * v_distance;
    return pn;
  };

  pcl::PointCloud<pcl::PointNormal> edges;
  const int N = lines.rows;
  for (int i = 0; i < N; i++) {
    cv::Mat xyxy = lines.row(i);
    Eigen::Vector2f xy1, xy2;
    xy1 << xyxy.at<float>(0), xyxy.at<float>(1);
    xy2 << xyxy.at<float>(2), xyxy.at<float>(3);

    auto pn_opt = conv(xy1, xy2);
    if (pn_opt.has_value()) edges.emplace_back(pn_opt.value());
  }

  // Draw projected edge image
  cv::Mat image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_8UC3);
  const cv::Size center(image.cols / 2, image.rows / 2);
  auto toCvPoint = [center, this](const Eigen::Vector3f & v) -> cv::Point {
    cv::Point pt;
    pt.x = -v.y() / this->max_range_ * center.width + center.width;
    pt.y = -v.x() / this->max_range_ * center.height + 2 * center.height;
    return pt;
  };

  pcl::PointCloud<pcl::PointNormal> long_edges;
  for (auto & pn : edges) {
    Eigen::Vector3f from = pn.getVector3fMap();
    Eigen::Vector3f to = pn.getNormalVector3fMap();
    float norm = (from - to).norm();
    cv::Scalar color = cv::Scalar(0, 255, 0);
    if (norm > length_threshold_) {
      long_edges.push_back(pn);
      color = cv::Scalar(0, 255, 255);
    }

    cv::line(
      image, toCvPoint(pn.getVector3fMap()), toCvPoint(pn.getNormalVector3fMap()), color, 2,
      cv::LineTypes::LINE_8);
  }

  // Publish
  publishCloud(long_edges, stamp);
  publishImage(image, stamp);
}

void LineSegmentDetector::publishImage(const cv::Mat & image, const rclcpp::Time & stamp) const
{
  cv_bridge::CvImage raw_image;
  raw_image.header.stamp = stamp;
  raw_image.header.frame_id = "map";
  raw_image.encoding = "bgr8";
  raw_image.image = image;
  pub_image_->publish(*raw_image.toImageMsg());
}

void LineSegmentDetector::publishCloud(
  const pcl::PointCloud<pcl::PointNormal> & cloud, const rclcpp::Time & stamp) const
{
  // Convert to msg
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  pub_cloud_->publish(msg);
}