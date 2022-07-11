#include "imgproc/segment_filter.hpp"

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace imgproc
{
SegmentFilter::SegmentFilter()
: Node("segment_filter"), subscriber_(rclcpp::Node::SharedPtr{this}, "lsd_cloud", "graph_segmented")
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  auto cb = std::bind(&SegmentFilter::execute, this, _1, _2);
  subscriber_.setCallback(cb);

  auto cb_info = [this](const CameraInfo & msg) -> void { info_ = msg; };
  sub_info_ =
    create_subscription<CameraInfo>("/sensing/camera/traffic_light/camera_info", 10, cb_info);
}

void SegmentFilter::execute(const PointCloud2 & lsd_msg, const PointCloud2 & segment_msg)
{
  if (!info_.has_value()) return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>()};
  pcl::fromROSMsg(segment_msg, *cloud);

  std::vector<cv::Point2i> contour;
  for (const pcl::PointXYZ & p : cloud->points) {
    contour.push_back(cv::Point2i(p.x, p.y));
  }
  std::vector<std::vector<cv::Point2i>> contours = {contour};

  cv::Size size(info_->width, info_->height);

  cv::Mat filt_image = cv::Mat::zeros(size, CV_8UC3);
  cv::drawContours(filt_image, contours, 0, cv::Scalar(0, 155, 155), -1);
  cv::imshow("a", filt_image);
  cv::waitKey(10);
}

}  // namespace imgproc

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imgproc::SegmentFilter>());
  rclcpp::shutdown();
  return 0;
}
