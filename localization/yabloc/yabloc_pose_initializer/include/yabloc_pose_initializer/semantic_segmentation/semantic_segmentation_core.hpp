#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

class SemanticSegmentationCore
{
public:
  SemanticSegmentationCore(const std::string & model_path)
  {
    net = cv::dnn::readNet(model_path);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }

  cv::Mat makeBlob(const cv::Mat & image)
  {
    double scale = 1.0;
    cv::Size size = cv::Size(896, 512);
    cv::Scalar mean = cv::Scalar(0, 0, 0);
    bool swap = true;
    bool crop = false;
    return cv::dnn::blobFromImage(image, scale, size, mean, swap, crop, CV_32F);
  }

  cv::Mat inference(const cv::Mat & image, double score_threshold = 0.5)
  {
    cv::Mat blob = makeBlob(image);
    net.setInput(blob);
    std::vector<std::string> output_layers = net.getUnconnectedOutLayersNames();
    std::vector<cv::Mat> masks;
    net.forward(masks, output_layers);

    cv::Mat mask = masks[0];
    cv::resize(mask, mask, cv::Size(image.cols, image.rows), 0, 0, cv::INTER_LINEAR);

    return normalize(mask, score_threshold);
  }

  cv::Mat normalize(const cv::Mat & mask, double score_threshold = 0.5)
  {
    std::vector<cv::Mat> masks;
    cv::split(mask, masks);
    std::vector<cv::Mat> bin_masks;

    for (size_t i = 1; i < masks.size(); ++i) {
      cv::Mat bin_mask;
      cv::threshold(masks[i], bin_mask, score_threshold, 255, cv::THRESH_BINARY_INV);
      bin_masks.push_back(255 - bin_mask);
    }

    cv::Mat result;
    cv::merge(bin_masks, result);
    return result;
  }

  cv::Mat drawOverlay(const cv::Mat & image, const cv::Mat & segmentation)
  {
    cv::Mat overlay_image = image.clone();
    return overlay_image * 0.5 + segmentation * 0.5;
  }

private:
  cv::dnn::Net net;
};