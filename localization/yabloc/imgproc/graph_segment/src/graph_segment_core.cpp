#include "graph_segment/graph_segment.hpp"
#include "graph_segment/histogram.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <pcdless_common/cv_decompress.hpp>
#include <pcdless_common/pub_sub.hpp>
#include <pcdless_common/timer.hpp>

#include <queue>

namespace pcdless::graph_segment
{
GraphSegment::GraphSegment()
: Node("graph_segment"),
  target_height_ratio_(declare_parameter<float>("target_height_ratio", 0.85)),
  target_candidate_box_width_(declare_parameter<int>("target_candidate_box_width", 15))
{
  using std::placeholders::_1;

  // Subscriber
  sub_image_ =
    create_subscription<Image>("src_image", 10, std::bind(&GraphSegment::on_image, this, _1));

  pub_mask_image_ = create_publisher<Image>("mask_image", 10);
  pub_debug_image_ = create_publisher<Image>("segmented_image", 10);

  const double sigma = declare_parameter<double>("sigma", 0.5);
  const float k = declare_parameter<float>("k", 300);
  const int min_size = declare_parameter<double>("min_size", 100);
  segmentation_ = cv::ximgproc::segmentation::createGraphSegmentation(sigma, k, min_size);
}

std::set<int> GraphSegment::search_similar_areas(
  const cv::Mat & rgb_image, const cv::Mat & segmented, int best_roadlike_class)
{
  std::unordered_map<int, Histogram> histogram_map;
  std::unordered_map<int, int> count_map;

  for (int h = 0; h < rgb_image.rows; h++) {
    const int * seg_ptr = segmented.ptr<int>(h);
    const cv::Vec3b * rgb_ptr = rgb_image.ptr<cv::Vec3b>(h);

    for (int w = 0; w < rgb_image.cols; w++) {
      int key = seg_ptr[w];
      cv::Vec3b rgb = rgb_ptr[w];
      if (count_map.count(key) == 0) {
        count_map[key] = 1;
        histogram_map[key].add(rgb);
      } else {
        count_map[key]++;
        histogram_map[key].add(rgb);
      }
    }
  }

  struct KeyAndArea
  {
    KeyAndArea(int key, int count) : key(key), count(count) {}
    int key;
    int count;
  };

  auto compare = [](KeyAndArea a, KeyAndArea b) { return a.count < b.count; };
  std::priority_queue<KeyAndArea, std::vector<KeyAndArea>, decltype(compare)> key_queue{compare};
  for (auto [key, count] : count_map) {
    key_queue.push({key, count});
  }

  Eigen::MatrixXf ref_histogram = histogram_map.at(best_roadlike_class).eval();

  std::cout << "histogram equality " << std::endl;

  int index = 0;
  std::set<int> acceptable_keys;
  while (!key_queue.empty()) {
    KeyAndArea key = key_queue.top();
    key_queue.pop();

    Eigen::MatrixXf query = histogram_map.at(key.key).eval();
    float score = Histogram::eval_histogram_intersection(ref_histogram, query);
    std::cout << " " << score;

    if (score > 0.8) acceptable_keys.insert(key.key);
    if (++index > 10) break;
  }
  std::cout << std::endl;

  // Visualilze
  cv::Mat new_segmented = rgb_image.clone();
  for (int h = 0; h < rgb_image.rows; h++) {
    const int * seg_ptr = segmented.ptr<int>(h);
    cv::Vec3b * rgb_ptr = new_segmented.ptr<cv::Vec3b>(h);

    for (int w = 0; w < rgb_image.cols; w++) {
      int key = seg_ptr[w];
      if (acceptable_keys.count(key)) rgb_ptr[w] = cv::Vec3b(0, 0, 255);
      if (key == best_roadlike_class) rgb_ptr[w] = cv::Vec3b(0, 255, 255);
    }
  }
  // cv::imshow("new_segmented", new_segmented);
  // cv::waitKey(10);

  return acceptable_keys;
}

cv::Vec3b random_hsv(int index)
{
  double base = (double)(index)*0.7071;
  return cv::Vec3b(fmod(base, 1.2) * 255, 0.7 * 255, 0.5 * 255);
};

int GraphSegment::search_most_road_like_class(const cv::Mat & segmented) const
{
  const int W = target_candidate_box_width_;
  const float R = target_height_ratio_;
  cv::Point2i target_px(segmented.cols * 0.5, segmented.rows * R);
  cv::Rect2i rect(target_px + cv::Point2i(-W, -W), target_px + cv::Point2i(W, W));

  std::unordered_map<int, int> areas;
  std::unordered_set<int> candidates;
  for (int h = 0; h < segmented.rows; h++) {
    const int * seg_ptr = segmented.ptr<int>(h);
    for (int w = 0; w < segmented.cols; w++) {
      int key = seg_ptr[w];
      if (areas.count(key) == 0) areas[key] = 0;
      areas[key]++;
      if (rect.contains(cv::Point2i{w, h})) candidates.insert(key);
    }
  }

  // Search the largest area and its class
  int max_area = 0;
  int max_area_class = -1;
  for (int c : candidates) {
    if (areas.at(c) < max_area) continue;
    max_area = areas.at(c);
    max_area_class = c;
  }
  return max_area_class;
}

void GraphSegment::on_image(const Image & msg)
{
  cv::Mat image = common::decompress_to_cv_mat(msg);
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);

  // Execute graph-based segmentation
  common::Timer timer;
  cv::Mat segmented;
  segmentation_->processImage(resized, segmented);
  RCLCPP_INFO_STREAM(get_logger(), "segmentation time: " << timer);

  //
  int target_class = search_most_road_like_class(segmented);
  //
  std::set<int> road_keys = search_similar_areas(resized, segmented, target_class);

  // Draw output image and debug image
  // TODO: use ptr instead of at()
  cv::Mat output_image = cv::Mat::zeros(resized.size(), CV_8UC1);
  cv::Mat debug_image = cv::Mat::zeros(resized.size(), CV_8UC3);
  for (int h = 0; h < resized.rows; h++) {
    for (int w = 0; w < resized.cols; w++) {
      cv::Point2i px(w, h);
      int key = segmented.at<int>(px);
      if (road_keys.count(key) > 0) {
        output_image.at<uchar>(px) = 255;
        if (key == target_class)
          debug_image.at<cv::Vec3b>(px) = cv::Vec3b(30, 255, 255);
        else
          debug_image.at<cv::Vec3b>(px) = cv::Vec3b(10, 255, 255);
      } else {
        debug_image.at<cv::Vec3b>(px) = random_hsv(key);
      }
    }
  }
  cv::cvtColor(debug_image, debug_image, cv::COLOR_HSV2BGR);
  cv::resize(output_image, output_image, image.size(), 0, 0, cv::INTER_NEAREST);
  cv::resize(debug_image, debug_image, image.size(), 0, 0, cv::INTER_NEAREST);

  common::publish_image(*pub_mask_image_, output_image, msg.header.stamp);

  draw_and_publish_image(image, debug_image, msg.header.stamp);
  RCLCPP_INFO_STREAM(get_logger(), "total processing time: " << timer);
}

void GraphSegment::draw_and_publish_image(
  const cv::Mat & raw_image, const cv::Mat & debug_image, const rclcpp::Time & stamp)
{
  cv::Mat show_image;
  cv::addWeighted(raw_image, 0.5, debug_image, 0.8, 1.0, show_image);
  const cv::Size size = debug_image.size();

  // Draw target rectangle
  {
    const int W = target_candidate_box_width_;
    const float R = target_height_ratio_;
    cv::Point2i target(size.width / 2, size.height * R);
    cv::Rect2i rect(target + cv::Point2i(-W, -W), target + cv::Point2i(W, W));
    cv::rectangle(show_image, rect, cv::Scalar::all(0), 2);
  }

  common::publish_image(*pub_debug_image_, show_image, stamp);
}

}  // namespace pcdless::graph_segment