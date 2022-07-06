#include "common/util.hpp"
#include "imgproc/graph_segment.hpp"

#include <opencv4/opencv2/imgproc.hpp>

namespace imgproc
{
GraphSegment::GraphSegment() : Node("graph_segment")
{
  using std::placeholders::_1;

  // Subscriber
  sub_image_ = create_subscription<Image>(
    "/sensing/camera/traffic_light/image_raw/compressed", 10,
    std::bind(&GraphSegment::callbackImage, this, _1));

  pub_cloud_ = create_publisher<PointCloud2>("/graph_segmented", 10);

  gs = cv::ximgproc::segmentation::createGraphSegmentation();
}

void GraphSegment::callbackImage(const Image & msg)
{
  cv::Mat image = util::decompress2CvMat(msg);
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
  cv::resize(output_image, output_image, image.size(), 0, 0, cv::INTER_NEAREST);

  // convert segmented area to polygon

  std::vector<std::vector<cv::Point2i>> contours;
  cv::findContours(output_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.size() == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "there are no contours");
    return;
  }
  auto & hull = contours.front();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const cv::Point2i p : hull) {
    pcl::PointXYZ xyz(p.x, p.y, 0);
    cloud.push_back(xyz);
  }

  util::publishCloud(*pub_cloud_, cloud, msg.header.stamp);
}

}  // namespace imgproc

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imgproc::GraphSegment>());
  rclcpp::shutdown();
  return 0;
}