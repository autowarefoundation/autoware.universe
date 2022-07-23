#include "sign_board/sign_detector.hpp"

#include "common/util.hpp"

#include <opencv4/opencv2/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace sign_board
{
SignDetector::SignDetector() : AbstCorrector("sign_detector"), tf_subscriber_(get_clock())
{
  using std::placeholders::_1;
  sub_sign_board_ = create_subscription<PointCloud2>(
    "/ll2_sign_board", 10,
    [this](const PointCloud2 & msg) -> void { pcl::fromROSMsg(msg, sign_board_); });

  auto cb_info = std::bind(&SignDetector::callbackInfo, this, _1);
  auto cb_image = std::bind(&SignDetector::callbackImage, this, _1);
  sub_info_ = create_subscription<CameraInfo>("/src_info", 10, cb_info);
  sub_image_ = create_subscription<Image>("/src_image", 10, cb_image);
}

void SignDetector::callbackImage(const Image & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "po");
  cv::Mat image = util::decompress2CvMat(msg);
  cv::imshow("test", image);
  cv::waitKey(5);

  const rclcpp::Time stamp = msg.header.stamp;
  auto array = getSyncronizedParticleArray(stamp);
  if (array.has_value()) {
    std::cout << "nice opt" << std::endl;
  } else {
    std::cout << "not opt" << std::endl;
  }
}

void SignDetector::callbackInfo(const CameraInfo & msg)
{
  info_ = msg;
  camera_extrinsic_ = tf_subscriber_(info_->header.frame_id, "base_link");
}

}  // namespace sign_board

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sign_board::SignDetector>());
  rclcpp::shutdown();
  return 0;
}
