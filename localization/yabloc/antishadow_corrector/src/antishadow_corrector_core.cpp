#include "antishadow_corrector/antishadow_corrector.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <vml_common/cv_decompress.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace modularized_particle_filter
{
AntishadowCorrector::AntishadowCorrector() : AbstCorrector("camera_particle_corrector")
{
  using std::placeholders::_1;
  auto lsd_callback = std::bind(&AntishadowCorrector::onLsd, this, _1);
  auto ll2_callback = std::bind(&AntishadowCorrector::onLl2, this, _1);
  sub_lsd_ = create_subscription<Image>("mapping_image", 10, lsd_callback);
  sub_ll2_ = create_subscription<PointCloud2>("ll2_road_marking", 10, ll2_callback);
}

void AntishadowCorrector::onLsd(const Image & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "LSD map is subscribed");

  cv::Mat lsd_image = vml_common::decompress2CvMat(msg);
  cv::imshow("a", lsd_image);
  cv::waitKey(1);

  const rclcpp::Time stamp = msg.header.stamp;
  std::optional<ParticleArray> opt_array = this->getSynchronizedParticleArray(stamp);
  if (!opt_array.has_value()) return;

  auto dt = (stamp - opt_array->header.stamp);
  if (std::abs(dt.seconds()) > 0.1)
    RCLCPP_WARN_STREAM(
      get_logger(), "Timestamp gap between image and particles is LARGE " << dt.seconds());

  // TODO:
}

void AntishadowCorrector::onLl2(const PointCloud2 & ll2_msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "LL2 cloud is subscribed");

  pcl::fromROSMsg(ll2_msg, ll2_cloud_);
}

}  // namespace modularized_particle_filter