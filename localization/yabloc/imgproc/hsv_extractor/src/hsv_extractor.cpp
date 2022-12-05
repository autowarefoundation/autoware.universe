#include "hsv_extractor/hsv_extractor.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <pcdless_common/cv_decompress.hpp>
#include <pcdless_common/pub_sub.hpp>
#include <pcdless_common/timer.hpp>

namespace pcdless::hsv_extractor
{
HsvExtractor::HsvExtractor()
: Node("hsv_extractor"),
  threshold_(declare_parameter<int>("threshold", 200)),
  saturation_gain_(declare_parameter<float>("saturation_gain", 1.0))
{
  using std::placeholders::_1;

  // Subscriber
  auto cb_image = std::bind(&HsvExtractor::on_image, this, _1);
  sub_image_ = create_subscription<Image>("src_image", 10, cb_image);

  // Publisher
  pub_image_lsd_ = create_publisher<Image>("extracted_image", 10);
}

void HsvExtractor::on_image(const sensor_msgs::msg::Image & msg)
{
  cv::Mat image = common::decompress_to_cv_mat(msg);
  execute(image, msg.header.stamp);
}

void HsvExtractor::execute(const cv::Mat & src_image, const rclcpp::Time & stamp)
{
  cv::Mat hsv_image;
  cv::cvtColor(src_image, hsv_image, cv::COLOR_BGR2HSV);

  std::vector<cv::Mat> hsv_images;
  cv::split(hsv_image, hsv_images);

  cv::Mat extracted_image;
  {
    cv::Mat sv_image = saturation_gain_ * hsv_images.at(1) + hsv_images.at(2);
    cv::imshow("sv", sv_image);
    cv::waitKey(10);

    cv::threshold(sv_image, extracted_image, threshold_, 255, cv::THRESH_BINARY);
    cv::Mat zero_image = cv::Mat::zeros(extracted_image.size(), CV_8UC1);
    cv::merge(std::vector<cv::Mat>{zero_image, extracted_image, extracted_image}, extracted_image);
  }

  cv::Mat show_image;
  cv::addWeighted(src_image, 0.5, extracted_image, 0.5, 1.0, show_image);
  common::publish_image(*pub_image_lsd_, show_image, stamp);
}

}  // namespace pcdless::hsv_extractor
