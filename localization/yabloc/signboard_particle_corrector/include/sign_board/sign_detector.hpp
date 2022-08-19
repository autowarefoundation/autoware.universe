#pragma once

#include <common/static_tf_subscriber.hpp>
#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vml_common/ground_plane.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

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
  using Float32Array = std_msgs::msg::Float32MultiArray;
  SignDetector();

private:
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<Float32Array>::SharedPtr sub_ground_plane_;
  common::StaticTfSubscriber tf_subscriber_;

  GroundPlane ground_plane_;
  std::optional<CameraInfo> info_;
  std::optional<Eigen::Affine3f> camera_extrinsic_;
  SignBoards sign_boards_;

  cv::Ptr<cv::ORB> feature_detector_;

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
}  // namespace sign_board