#pragma once

#include "common/static_tf_subscriber.hpp"

#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sign_board
{
class SignDetector : public AbstCorrector
{
public:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using SignBoard = lanelet::ConstLineString3d;
  using SignBoards = lanelet::ConstLineStrings3d;
  SignDetector();

private:
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  common::StaticTfSubscriber tf_subscriber_;

  std::optional<CameraInfo> info_;
  std::optional<Eigen::Affine3f> camera_extrinsic_;
  SignBoards sign_boards_;

  struct Contour
  {
    signed long id;
    std::vector<cv::Point2i> polygon;
  };

  SignBoards extractSpecifiedLineString(
    const lanelet::LineStringLayer & line_string_layer,
    const std::set<std::string> & visible_labels);

  std::optional<Contour> extractSignBoardContour(
    const ParticleArray & array, const SignBoard board);

  void callbackImage(const Image & image);
  void callbackVectorMap(const HADMapBin & map_bin);
  void callbackInfo(const CameraInfo & info);
};
};  // namespace sign_board