#include "vmvl_imgproc/lsd.hpp"

#include <vml_common/cv_decompress.hpp>
#include <vml_common/pub_sub.hpp>
#include <vml_common/timer.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace imgproc
{
LineSegmentDetector::LineSegmentDetector()
: Node("line_detector"),
  truncate_pixel_threshold_(declare_parameter<int>("truncate_pixel_threshold", -1))
{
  using std::placeholders::_1;

  // Subscriber
  auto cb_image = std::bind(&LineSegmentDetector::imageCallback, this, _1);
  sub_image_ =
    create_subscription<Image>("/sensing/camera/traffic_light/image_raw/compressed", 10, cb_image);

  // Publisher
  pub_image_lsd_ = create_publisher<Image>("lsd_image", 10);
  pub_cloud_ = create_publisher<PointCloud2>("lsd_cloud", 10);

  lsd_ =
    cv::lsd::createLineSegmentDetector(cv::lsd::LSD_REFINE_STD, 0.8, 0.6, 2.0, 22.5, 0, 0.7, 1024);
}

void LineSegmentDetector::imageCallback(const sensor_msgs::msg::Image & msg)
{
  cv::Mat image = vml_common::decompress2CvMat(msg);
  execute(image, msg.header.stamp);
}

void LineSegmentDetector::execute(const cv::Mat & image, const rclcpp::Time & stamp)
{
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

  cv::Mat lines;
  {
    Timer timer;
    lsd_->detect(gray_image, lines);
    lsd_->drawSegments(gray_image, lines);
    RCLCPP_INFO_STREAM(this->get_logger(), "lsd: " << timer);
  }

  vml_common::publishImage(*pub_image_lsd_, gray_image, stamp);

  pcl::PointCloud<pcl::PointNormal> line_cloud;
  std::vector<cv::Mat> filtered_lines = removeTooOuterElements(lines, image.size());

  for (const cv::Mat & xy_xy : filtered_lines) {
    Eigen::Vector3f xy1, xy2;
    xy1 << xy_xy.at<float>(0), xy_xy.at<float>(1), 0;
    xy2 << xy_xy.at<float>(2), xy_xy.at<float>(3), 0;
    pcl::PointNormal pn;
    pn.getVector3fMap() = xy1;
    pn.getNormalVector3fMap() = xy2;
    line_cloud.push_back(pn);
  }
  vml_common::publishCloud(*pub_cloud_, line_cloud, stamp);
}

std::vector<cv::Mat> LineSegmentDetector::removeTooOuterElements(
  const cv::Mat & lines, const cv::Size & size) const
{
  std::vector<cv::Rect2i> rect_vector;
  rect_vector.emplace_back(0, 0, size.width, 3);
  rect_vector.emplace_back(0, size.height - 3, size.width, 3);
  rect_vector.emplace_back(0, 0, 3, size.height);
  rect_vector.emplace_back(size.width - 3, 0, 3, size.height);

  std::vector<cv::Mat> filtered_lines;
  for (int i = 0; i < lines.rows; i++) {
    cv::Mat xy_xy = lines.row(i);
    cv::Point2f xy1(xy_xy.at<float>(0), xy_xy.at<float>(1));
    cv::Point2f xy2(xy_xy.at<float>(2), xy_xy.at<float>(3));

    bool enabled = true;
    for (const cv::Rect2i & rect : rect_vector) {
      if (rect.contains(xy1) && rect.contains(xy2)) {
        enabled = false;
        break;
      }
    }
    if (enabled) filtered_lines.push_back(xy_xy);
  }

  return filtered_lines;
}

}  // namespace imgproc
