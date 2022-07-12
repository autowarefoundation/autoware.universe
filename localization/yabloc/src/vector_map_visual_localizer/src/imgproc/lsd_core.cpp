#include "common/timer.hpp"
#include "common/util.hpp"
#include "imgproc/lsd.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace imgproc
{
LineSegmentDetector::LineSegmentDetector() : Node("line_detector")
{
  using std::placeholders::_1;

  // Subscriber
  sub_image_ = create_subscription<Image>(
    "/sensing/camera/traffic_light/image_raw/compressed", 10,
    std::bind(&LineSegmentDetector::imageCallback, this, _1));

  // Publisher
  pub_image_lsd_ = create_publisher<Image>("/lsd_image", 10);
  pub_cloud_ = create_publisher<PointCloud2>("/lsd_cloud", 10);

  lsd_ =
    cv::lsd::createLineSegmentDetector(cv::lsd::LSD_REFINE_STD, 0.8, 0.6, 2.0, 22.5, 0, 0.7, 1024);
  gs_ = cv::ximgproc::segmentation::createGraphSegmentation();
}

void LineSegmentDetector::imageCallback(const sensor_msgs::msg::Image & msg)
{
  cv::Mat image = util::decompress2CvMat(msg);
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

  // OBSL: We will remove this function soon
  cv::Mat segmented = segmentationGraph(image);

  {
    cv::Mat rgb_segmented;
    cv::merge(
      std::vector<cv::Mat>{cv::Mat::zeros(segmented.size(), CV_8UC1), segmented, segmented},
      rgb_segmented);
    cv::addWeighted(gray_image, 0.8, rgb_segmented, 0.5, 1, gray_image);
    util::publishImage(*pub_image_lsd_, gray_image, stamp);
  }

  pcl::PointCloud<pcl::PointNormal> line_cloud;
  for (int i = 0; i < lines.rows; i++) {
    cv::Mat xy_xy = lines.row(i);
    Eigen::Vector3f xy1, xy2;
    xy1 << xy_xy.at<float>(0), xy_xy.at<float>(1), 0;
    xy2 << xy_xy.at<float>(2), xy_xy.at<float>(3), 0;
    pcl::PointNormal pn;
    pn.getVector3fMap() = xy1;
    pn.getNormalVector3fMap() = xy2;
    line_cloud.push_back(pn);
  }
  util::publishCloud(*pub_cloud_, line_cloud, stamp);
}

cv::Mat LineSegmentDetector::segmentationGraph(const cv::Mat & image)
{
  Timer t;
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);

  cv::Mat segmented;
  gs_->processImage(resized, segmented);

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
}  // namespace imgproc
