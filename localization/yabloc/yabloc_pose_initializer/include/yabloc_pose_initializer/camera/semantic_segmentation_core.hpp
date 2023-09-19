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

  cv::Mat convert_blob_to_image(const cv::Mat & blob)
  {
    if (blob.size.dims() != 4) {
      throw std::runtime_error("blob has invalid size");
    }
    const int channels = blob.size[1];
    const int height = blob.size[2];
    const int width = blob.size[3];
    cv::Mat image = cv::Mat(height, width, CV_32FC4);

    for (int h = 0; h < height; ++h) {
      for (int w = 0; w < width; ++w) {
        cv::Vec4f vec4f;
        for (int c = 0; c < channels; ++c) {
          vec4f[c] = blob.at<float>(c * height * width + h * width + w);
        }
        image.at<cv::Vec4f>(h, w) = vec4f;
      }
    }
    return image;
  }

  cv::Mat inference(const cv::Mat & image, double score_threshold = 0.5)
  {
    std::cout << "DEBUG image" << image.size() << std::endl;
    cv::Mat blob = makeBlob(image);
    net.setInput(blob);
    std::vector<std::string> output_layers = net.getUnconnectedOutLayersNames();
    std::vector<cv::Mat> masks;
    net.forward(masks, output_layers);

    std::cout << "DEBUG mask " << masks.size() << std::endl;
    cv::Mat mask = masks[0];
    cv::Mat output = convert_blob_to_image(mask);

    std::cout << "DEBUG output " << output.size() << " " << output.channels() << std::endl;
    cv::resize(output, output, cv::Size(image.cols, image.rows), 0, 0, cv::INTER_LINEAR);

    return normalize(output, score_threshold);
  }

  cv::Mat normalize(const cv::Mat & mask, double score_threshold = 0.5)
  {
    std::cout << "DEBUG normalize mask:" << mask.size() << std::endl;

    std::vector<cv::Mat> masks;
    cv::split(mask, masks);
    std::vector<cv::Mat> bin_masks;

    for (size_t i = 1; i < masks.size(); ++i) {
      cv::Mat bin_mask;
      cv::threshold(masks[i], bin_mask, score_threshold, 255, cv::THRESH_BINARY_INV);
      bin_mask.convertTo(bin_mask, CV_8UC1);
      bin_masks.push_back(255 - bin_mask);
    }

    std::cout << "DEBUG bin_masks" << masks.size() << std::endl;

    cv::Mat result;
    cv::merge(bin_masks, result);
    cv::imshow("bin_masks", result);
    cv::waitKey(0);
    return result;
  }

  cv::Mat drawOverlay(const cv::Mat & image, const cv::Mat & segmentation)
  {
    cv::Mat overlay_image = image.clone();
    return overlay_image * 0.5 + segmentation * 0.5;
  }

  static void print_error_message(const rclcpp::Logger & logger)
  {
    const std::string ERROR_MESSAGE =
      R"(The yabloc_pose_initializer is not working correctly because the DNN model has not been downloaded correctly.
To download models, "-DDOWNLOAD_ARTIFACTS=ON" is required at build time.
Please see the README of yabloc_pose_initializer for more information.)";

    std::istringstream stream(ERROR_MESSAGE);
    std::string line;
    while (std::getline(stream, line)) {
      RCLCPP_ERROR_STREAM(logger, line);
    }
  }

private:
  cv::dnn::Net net;
};