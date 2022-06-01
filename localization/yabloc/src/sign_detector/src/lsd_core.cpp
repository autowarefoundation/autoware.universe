#include "sign_detector/lsd.hpp"
#include "sign_detector/timer.hpp"
#include "sign_detector/util.hpp"

#include <pcl_conversions/pcl_conversions.h>

void LineSegmentDetector::compressedImageCallback(const sensor_msgs::msg::CompressedImage & msg)
{
  cv::Mat image = decompress2CvMat(msg);
  execute(image, msg.header.stamp);
}

void LineSegmentDetector::imageCallback(const sensor_msgs::msg::Image & msg)
{
  cv::Mat image = decompress2CvMat(msg);
  execute(image, msg.header.stamp);
}

void LineSegmentDetector::execute(const cv::Mat & image, const rclcpp::Time & stamp)
{
  if (!info_.has_value()) return;
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

  std::chrono::time_point start = std::chrono::system_clock::now();
  cv::Mat lines;
  {
    Timer timer;
    lsd->detect(gray_image, lines);
    lsd->drawSegments(gray_image, lines);
    RCLCPP_INFO_STREAM(this->get_logger(), "lsd: " << timer.microSeconds() / 1000.0f << "ms");
  }

  auto dur = std::chrono::system_clock::now() - start;
  long ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();

  cv::Mat segmented = segmentationGraph(image);
  cv::Mat gray_segmented;
  cv::cvtColor(segmented, gray_segmented, cv::COLOR_GRAY2BGR);
  cv::addWeighted(gray_image, 0.8, gray_segmented, 0.5, 1, gray_image);

  // Publish lsd image
  publishImage(*pub_image_lsd_, gray_image, stamp);

  {
    Timer project_timer;
    cv::Mat K = cv::Mat(cv::Size(3, 3), CV_64FC1, (void *)(info_->k.data()));
    projectEdgeOnPlane(lines, K, stamp, segmented);
    RCLCPP_INFO_STREAM(
      this->get_logger(), "projection: " << project_timer.microSeconds() / 1000.0f << "ms");
  }
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
  const cv::Mat & lines, const cv::Mat & K_cv, const rclcpp::Time & stamp,
  const cv::Mat & mask) const
{
  if (!camera_extrinsic_.has_value()) {
    RCLCPP_WARN_STREAM(this->get_logger(), "camera_extrinsic_ has not been initialized");
    return;
  }
  Eigen::Vector3f t = camera_extrinsic_->translation();
  Eigen::Quaternionf q(camera_extrinsic_->rotation());

  // Convert to projected coordinate
  Eigen::Matrix3f K, K_inv;
  cv::cv2eigen(K_cv, K);
  K_inv = K.inverse();

  auto conv = [t, q, K_inv](const Eigen::Vector2f & u) -> std::optional<Eigen::Vector3f> {
    Eigen::Vector3f u3(u.x(), u.y(), 1);
    Eigen::Vector3f u_bearing = (q * K_inv * u3).normalized();
    if (u_bearing.z() > -0.01) return std::nullopt;
    float u_distance = -t.z() / u_bearing.z();
    Eigen::Vector3f v;
    v.x() = t.x() + u_bearing.x() * u_distance;
    v.y() = t.y() + u_bearing.y() * u_distance;
    v.z() = 0;
    return v;
  };

  pcl::PointCloud<pcl::PointNormal> edges;
  const int N = lines.rows;
  for (int i = 0; i < N; i++) {
    cv::Mat xyxy = lines.row(i);
    Eigen::Vector2f xy1, xy2;
    xy1 << xyxy.at<float>(0), xyxy.at<float>(1);
    xy2 << xyxy.at<float>(2), xyxy.at<float>(3);

    // TODO:
    // cv::Point2i u1(xy1.x(), xy1.y());
    // cv::Point2i u2(xy2.x(), xy2.y());
    // bool flag = (mask.at<cv::Vec3b>(u1)[1] == 0) & (mask.at<cv::Vec3b>(u2)[1] == 0);
    // if (flag) continue;

    auto p1_opt = conv(xy1);
    auto p2_opt = conv(xy2);
    if (p1_opt.has_value() & p2_opt.has_value()) {
      pcl::PointNormal pn;
      pn.getVector3fMap() = p1_opt.value();
      pn.getNormalVector3fMap() = p2_opt.value();
      edges.emplace_back(pn);
    }
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

  {
    // Compute convex hull
    std::vector<std::vector<cv::Point2i>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() == 0) {
      RCLCPP_WARN_STREAM(this->get_logger(), "there are no contours");
      return;
    }
    // std::vector<cv::Point> hull(contours.size());
    // cv::convexHull(contours.front(), hull);
    auto & hull = contours.front();
    std::vector<std::vector<cv::Point2i>> projected_hull(1);
    for (int i = 0; i < hull.size(); i++) {
      Eigen::Vector2f u(hull.at(i).x, hull.at(i).y);
      auto v_opt = conv(u);
      if (v_opt.has_value()) projected_hull.front().push_back(toCvPoint(v_opt.value()));
    }
    cv::drawContours(image, projected_hull, 0, cv::Scalar(0, 155, 155), -1);
  }

  pcl::PointCloud<pcl::PointNormal> reliable_edges;
  for (auto & pn : edges) {
    cv::Scalar color = cv::Scalar(255, 255, 0);
    cv::line(
      image, toCvPoint(pn.getVector3fMap()), toCvPoint(pn.getNormalVector3fMap()), color, 2,
      cv::LineTypes::LINE_8);
  }

  // Publish
  publishCloud(reliable_edges, stamp);
  publishImage(*pub_image_, image, stamp);
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

cv::Mat LineSegmentDetector::segmentationGraph(const cv::Mat & image)
{
  if (gs.empty()) gs = cv::ximgproc::segmentation::createGraphSegmentation();

  Timer t;
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);

  cv::Mat segmented;
  gs->processImage(resized, segmented);

  // TODO: THIS IS EFFICIENT BUT STUPID
  int target_class = segmented.at<int>(cv::Point2i(resized.cols / 2, resized.rows * 0.8));

  cv::Mat output_image = cv::Mat::zeros(resized.size(), CV_8UC1);
  for (int w = 0; w < resized.cols; w++) {
    for (int h = 0; h < resized.rows; h++) {
      cv::Point2i px(w, h);
      if (segmented.at<int>(px) == target_class) output_image.at<uchar>(px) = 255;
    }
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "segmentation: " << t.microSeconds() / 1000.0f << "ms");
  cv::resize(output_image, output_image, image.size(), 0, 0, cv::INTER_NEAREST);
  return output_image;
}
