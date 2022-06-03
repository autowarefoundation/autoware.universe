#include "sign_detector/optflow.hpp"
#include "sign_detector/util.hpp"

#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <sophus/geometry.hpp>

#include <cv_bridge/cv_bridge.h>

void OptFlowNode::computeRotation(
  const std::vector<cv::Point2f> & pt1, const std::vector<cv::Point2f> & pt2)
{
  if (!scaled_intrinsic.has_value()) return;
  const cv::Mat K = scaled_intrinsic.value();

  cv::Mat R, t, mask;
  cv::Mat E = cv::findEssentialMat(pt1, pt2, K, cv::RANSAC, 0.999, 1.0, mask);
  cv::recoverPose(E, pt1, pt2, K, R, t, mask);

  Eigen::Matrix3f R_eigen;
  cv::cv2eigen(R, R_eigen);

  Eigen::Vector3f omega = Sophus::SO3f(R_eigen).log();
  std::cout << omega.norm() << std::endl;
}

void OptFlowNode::trackOptFlow(const cv::Mat & src_image)
{
  constexpr bool MUTUAL_CHECK = true;

  RCLCPP_INFO_STREAM(this->get_logger(), "tracked points: " << p0.size());

  cv::Mat show_image = src_image.clone();
  cv::Mat gray_image;
  cv::cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);

  // ================================================
  // First find good features
  if (!last_gray_image.has_value()) {
    cv::goodFeaturesToTrack(gray_image, p0, 100, 0.3, 7, cv::Mat{}, 7, false, 0.04);
    last_gray_image = gray_image;
    return;
  }

  // ================================================
  cv::TermCriteria criteria =
    cv::TermCriteria((cv::TermCriteria::COUNT | cv::TermCriteria::EPS), 10, 0.03);
  std::vector<uchar> status;
  std::vector<float> err;
  std::vector<cv::Point2f> p1;
  cv::calcOpticalFlowPyrLK(
    last_gray_image.value(), gray_image, p0, p1, status, err, cv::Size(30, 30), 2, criteria);

  std::vector<cv::Point2f> enable_p0;
  std::vector<cv::Point2f> enable_p1;
  for (uint i = 0; i < p0.size(); i++) {
    if (status[i] == 1) {
      enable_p0.push_back(p0[i]);
      enable_p1.push_back(p1[i]);
    }
  }

  if (MUTUAL_CHECK) {
    std::vector<uchar> reverse_status;
    std::vector<float> reverse_err;
    std::vector<cv::Point2f> reverse_p0;
    cv::calcOpticalFlowPyrLK(
      gray_image, last_gray_image.value(), enable_p1, reverse_p0, reverse_status, reverse_err,
      cv::Size(30, 30), 2, criteria);
    std::vector<cv::Point2f> more_enable_p0;
    std::vector<cv::Point2f> more_enable_p1;
    for (uint i = 0; i < enable_p1.size(); i++) {
      if (reverse_status[i] == 1) {
        cv::Point2f d = enable_p0[i] - reverse_p0[i];
        if (d.x * d.x + d.y * d.y < 10) {
          more_enable_p0.push_back(enable_p0[i]);
          more_enable_p1.push_back(enable_p1[i]);
        }
      }
    }
    enable_p0 = more_enable_p0;
    enable_p1 = more_enable_p1;
  }

  for (uint i = 0; i < enable_p1.size(); i++) {
    cv::line(show_image, enable_p1[i], enable_p0[i], cv::Scalar(0, 0, 255), 1);
    cv::circle(show_image, enable_p1[i], 4, cv::Scalar(0, 255, 255), 1);
  }
  cv::imshow("optflow", show_image);
  cv::waitKey(1);

  computeRotation(enable_p0, enable_p1);

  // ================================================
  // Pickup additional good features
  {
    if (enable_p1.size() < 50) {
      cv::Mat mask = cv::Mat::ones(gray_image.size(), CV_8UC1);
      for (const auto p : enable_p1) cv::circle(mask, p, 20, cv::Scalar::all(0), -1);

      std::vector<cv::Point2f> additional_p1;
      cv::goodFeaturesToTrack(gray_image, additional_p1, 50, 0.3, 7, mask, 7, false, 0.04);
      std::copy(additional_p1.begin(), additional_p1.end(), std::back_inserter(enable_p1));
    }
  }

  p0 = enable_p1;
  last_gray_image = gray_image;
}

void OptFlowNode::imageCallback(const sensor_msgs::msg::CompressedImage & msg)
{
  cv::Mat image = decompress2CvMat(msg);
  if (!info_.has_value()) return;
  cv::Mat K = cv::Mat(cv::Size(3, 3), CV_64FC1, (void *)(info_->k.data()));
  cv::Mat D = cv::Mat(cv::Size(5, 1), CV_64FC1, (void *)(info_->d.data()));
  cv::Mat undistorted;
  cv::undistort(image, undistorted, K, D, K);
  image = undistorted;

  cv::Size size = image.size();
  const int WIDTH = 800;
  const float SCALE = 1.0f * WIDTH / size.width;
  int HEIGHT = SCALE * size.height;
  cv::resize(image, image, cv::Size(WIDTH, HEIGHT));
  scaled_intrinsic = SCALE * K;
  scaled_intrinsic->at<double>(2, 2) = 1;

  trackOptFlow(image);
  publishImage(*pub_image_, image, this->get_clock()->now());
}
