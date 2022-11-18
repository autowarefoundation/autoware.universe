#include "graph_segment/graph_segment.hpp"

#include <opencv4/opencv2/imgproc.hpp>
#include <pcdless_common/cv_decompress.hpp>
#include <pcdless_common/pub_sub.hpp>

namespace pcdless::graph_segment
{
GraphSegment::GraphSegment()
: Node("graph_segment"),
  target_height_ratio_(declare_parameter<float>("target_height_ratio", 0.85)),
  target_candidate_box_width_(declare_parameter<int>("target_candidate_box_width", 15))
{
  using std::placeholders::_1;

  // Subscriber
  sub_image_ = create_subscription<Image>(
    "/sensing/camera/traffic_light/image_raw/compressed", 10,
    std::bind(&GraphSegment::on_image, this, _1));

  pub_cloud_ = create_publisher<PointCloud2>("graph_segmented", 10);
  pub_image_ = create_publisher<Image>("segmented_image", 10);

  segmentation_ = cv::ximgproc::segmentation::createGraphSegmentation();
}

void GraphSegment::on_image(const Image & msg)
{
  cv::Mat image = common::decompress_to_cv_mat(msg);
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);

  cv::Mat segmented;
  segmentation_->processImage(resized, segmented);

  int target_class = -1;
  {
    const int W = target_candidate_box_width_;
    const float R = target_height_ratio_;
    cv::Point2i target_px(resized.cols * 0.5, resized.rows * R);
    cv::Rect2i rect(target_px + cv::Point2i(-W, -W), target_px + cv::Point2i(W, W));

    std::unordered_map<int, int> areas;
    std::unordered_set<int> candidates;

    for (int h = 0; h < resized.rows; h++) {
      int * seg_ptr = segmented.ptr<int>(h);
      for (int w = 0; w < resized.cols; w++) {
        int key = seg_ptr[w];
        if (areas.count(key) == 0) areas[key] = 0;
        areas[key]++;
        if (rect.contains(cv::Point2i{w, h})) candidates.insert(key);
      }
    }
    int max_area = 0;
    int max_area_id = -1;
    for (int c : candidates) {
      if (areas.at(c) < max_area) continue;
      max_area = areas.at(c);
      max_area_id = c;
    }
    target_class = max_area_id;
  }

  cv::Mat output_image = cv::Mat::zeros(resized.size(), CV_8UC1);
  for (int h = 0; h < resized.rows; h++) {
    for (int w = 0; w < resized.cols; w++) {
      cv::Point2i px(w, h);
      if (segmented.at<int>(px) == target_class) output_image.at<uchar>(px) = 255;
    }
  }
  cv::resize(output_image, output_image, image.size(), 0, 0, cv::INTER_NEAREST);

  // Convert segmented area to polygon
  std::vector<std::vector<cv::Point2i>> contours;
  cv::findContours(output_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.size() == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "there are no contours");
    return;
  }
  auto & hull = contours.front();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const cv::Point2i & p : hull) {
    pcl::PointXYZ xyz(p.x, p.y, 0);
    cloud.push_back(xyz);
  }

  common::publish_cloud(*pub_cloud_, cloud, msg.header.stamp);

  publish_image(image, segmented, msg.header.stamp, target_class);
}

void GraphSegment::publish_image(
  const cv::Mat & raw_image, const cv::Mat & segmentation, const rclcpp::Time & stamp,
  int target_class)
{
  const cv::Size size = segmentation.size();

  auto randomHsv = [](int index) -> cv::Scalar {
    double base = (double)(index)*0.618033988749895 + 0.24443434;
    return cv::Scalar(fmod(base, 1.2) * 255, 0.70 * 255, 0.5 * 255);
  };

  cv::Mat segmented_image = cv::Mat::zeros(size, CV_8UC3);
  for (int i = 0; i < segmented_image.rows; i++) {
    const int * p = segmentation.ptr<int>(i);
    uchar * p2 = segmented_image.ptr<uchar>(i);
    for (int j = 0; j < segmented_image.cols; j++) {
      cv::Scalar color(30, 255, 255);
      if (p[j] != target_class) color = randomHsv(p[j]);
      p2[j * 3] = (uchar)color[0];
      p2[j * 3 + 1] = (uchar)color[1];
      p2[j * 3 + 2] = (uchar)color[2];
    }
  }
  cv::cvtColor(segmented_image, segmented_image, cv::COLOR_HSV2BGR);

  {
    const int W = target_candidate_box_width_;
    const float R = target_height_ratio_;
    cv::Point2i target(size.width / 2, size.height * R);
    cv::Rect2i rect(target + cv::Point2i(-W, -W), target + cv::Point2i(W, W));
    cv::rectangle(segmented_image, rect, cv::Scalar::all(0), 2);
  }

  cv::resize(segmented_image, segmented_image, raw_image.size(), 0, 0, cv::INTER_NEAREST);

  cv::Mat show_image;
  cv::addWeighted(raw_image, 0.5, segmented_image, 0.8, 1.0, show_image);
  common::publish_image(*pub_image_, show_image, stamp);
}

}  // namespace pcdless::graph_segment