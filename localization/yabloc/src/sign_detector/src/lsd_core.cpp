#include "common/util.hpp"
#include "sign_detector/lsd.hpp"
#include "sign_detector/timer.hpp"

#include <pcl_conversions/pcl_conversions.h>

cv::Point2i LineSegmentDetector::toCvPoint(const Eigen::Vector3f & v) const
{
  cv::Point pt;
  pt.x = -v.y() / max_range_ * image_size_ * 0.5f + image_size_ / 2;
  pt.y = -v.x() / max_range_ * image_size_ * 0.5f + image_size_;
  return pt;
}

void LineSegmentDetector::compressedImageCallback(const sensor_msgs::msg::CompressedImage & msg)
{
  cv::Mat image = util::decompress2CvMat(msg);
  execute(image, msg.header.stamp);
}

void LineSegmentDetector::imageCallback(const sensor_msgs::msg::Image & msg)
{
  cv::Mat image = util::decompress2CvMat(msg);
  execute(image, msg.header.stamp);
}

void LineSegmentDetector::execute(const cv::Mat & image, const rclcpp::Time & stamp)
{
  if (!info_.has_value()) return;
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

  cv::Mat lines;
  {
    Timer timer;
    lsd->detect(gray_image, lines);
    lsd->drawSegments(gray_image, lines);
    RCLCPP_INFO_STREAM(this->get_logger(), "lsd: " << timer);
  }

  cv::Mat segmented = segmentationGraph(image);

  {
    cv::Mat rgb_segmented;
    cv::merge(
      std::vector<cv::Mat>{cv::Mat::zeros(segmented.size(), CV_8UC1), segmented, segmented},
      rgb_segmented);
    cv::addWeighted(gray_image, 0.8, rgb_segmented, 0.5, 1, gray_image);
    util::publishImage(*pub_image_lsd_, gray_image, stamp);
  }
  {
    Timer project_timer;
    cv::Mat K = cv::Mat(cv::Size(3, 3), CV_64FC1, (void *)(info_->k.data()));
    projectEdgeOnPlane(lines, K, stamp, segmented);
    RCLCPP_INFO_STREAM(this->get_logger(), "projection: " << project_timer.microSeconds());
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

std::set<ushort> getUniquePixelValue(cv::Mat & image)
{
  auto begin = image.begin<ushort>();
  auto last = std::unique(begin, image.end<ushort>());
  std::sort(begin, last);
  last = std::unique(begin, last);
  return std::set<ushort>(begin, last);
}

void LineSegmentDetector::projectEdgeOnPlane(
  const cv::Mat & lines, const cv::Mat & K_cv, const rclcpp::Time & stamp,
  const cv::Mat & mask) const
{
  if (!camera_extrinsic_.has_value()) {
    RCLCPP_WARN_STREAM(this->get_logger(), "camera_extrinsic_ has not been initialized");
    return;
  }
  const Eigen::Vector3f t = camera_extrinsic_->translation();
  const Eigen::Quaternionf q(camera_extrinsic_->rotation());

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
    cv::Mat xy_xy = lines.row(i);
    Eigen::Vector2f xy1, xy2;
    xy1 << xy_xy.at<float>(0), xy_xy.at<float>(1);
    xy2 << xy_xy.at<float>(2), xy_xy.at<float>(3);

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
  cv::Mat mask_image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_16UC1);
  std::vector<std::vector<cv::Point2i>> projected_hull(1);
  {
    // Compute convex hull
    std::vector<std::vector<cv::Point2i>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() == 0) {
      RCLCPP_WARN_STREAM(this->get_logger(), "there are no contours");
      return;
    }
    auto & hull = contours.front();
    for (int i = 0; i < hull.size(); i++) {
      Eigen::Vector2f u(hull.at(i).x, hull.at(i).y);
      auto v_opt = conv(u);
      if (v_opt.has_value()) projected_hull.front().push_back(toCvPoint(v_opt.value()));
    }
    auto max_color = cv::Scalar::all(std::numeric_limits<u_short>::max());
    cv::drawContours(mask_image, projected_hull, 0, max_color, -1);
  }

  cv::Mat line_image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_16UC1);
  for (int i = 0; i < edges.size(); i++) {
    auto & pn = edges.at(i);
    cv::Point2i p1 = toCvPoint(pn.getVector3fMap());
    cv::Point2i p2 = toCvPoint(pn.getNormalVector3fMap());
    cv::Scalar color = cv::Scalar::all(i + 1);
    cv::line(line_image, p1, p2, color, 1, cv::LineTypes::LINE_4);
  }

  cv::Mat masked_line;
  cv::bitwise_and(mask_image, line_image, masked_line);
  std::set<ushort> pixel_values = getUniquePixelValue(masked_line);

  pcl::PointCloud<pcl::PointNormal> reliable_edges;
  cv::Mat reliable_line_image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_8UC3);
  cv::drawContours(reliable_line_image, projected_hull, 0, cv::Scalar(0, 155, 155), -1);
  for (int i = 0; i < edges.size(); i++) {
    auto & pn = edges.at(i);
    cv::Point2i p1 = toCvPoint(pn.getVector3fMap());
    cv::Point2i p2 = toCvPoint(pn.getNormalVector3fMap());
    cv::Scalar color = cv::Scalar(255, 255, 255);
    if (pixel_values.count(i + 1) != 0) {
      color = cv::Scalar(100, 100, 255);
      reliable_edges.push_back(pn);
    }
    cv::line(reliable_line_image, p1, p2, color, 2, cv::LineTypes::LINE_8);
  }

  // Publish
  publishCloud(reliable_edges, stamp);
  util::publishImage(*pub_image_, reliable_line_image, stamp);
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
  RCLCPP_INFO_STREAM(this->get_logger(), "segmentation: " << t);
  cv::resize(output_image, output_image, image.size(), 0, 0, cv::INTER_NEAREST);
  return output_image;
}
