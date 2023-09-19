#include <opencv2/core.hpp>
#include <rclcpp/logger.hpp>

#include <iostream>

class SemanticSegmentation
{
public:
  SemanticSegmentation(const std::string & model_path);

  cv::Mat inference(const cv::Mat & image, double score_threshold = 0.5);

  static void print_error_message(const rclcpp::Logger & logger);

private:
  cv::Mat makeBlob(const cv::Mat & image);

  cv::Mat convert_blob_to_image(const cv::Mat & blob);

  cv::Mat normalize(const cv::Mat & mask, double score_threshold = 0.5);

  cv::Mat drawOverlay(const cv::Mat & image, const cv::Mat & segmentation);

  struct Impl;
  std::shared_ptr<Impl> impl_{nullptr};
};