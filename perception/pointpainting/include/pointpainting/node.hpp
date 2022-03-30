// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POINTPAINTING__NODE_HPP_
#define POINTPAINTING__NODE_HPP_

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <centerpoint_trt.hpp>
#include <config.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <boost/circular_buffer.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace pointpainting
{
using Label = autoware_auto_perception_msgs::msg::ObjectClassification;
class Debugger
{
public:
  explicit Debugger(rclcpp::Node * node, const int camera_num);
  ~Debugger() = default;
  rclcpp::Node * node_;
  void showImage(
    const int id, const rclcpp::Time & time,
    const std::vector<sensor_msgs::msg::RegionOfInterest> & image_rois,
    // const std::vector<sensor_msgs::msg::RegionOfInterest> & pointcloud_rois,
    const std::vector<Eigen::Vector2d> & points);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg, const int id);
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  std::vector<image_transport::Subscriber> image_subs_;
  std::vector<image_transport::Publisher> image_pubs_;
  std::vector<boost::circular_buffer<sensor_msgs::msg::Image::ConstSharedPtr>> image_buffers_;
};

class PointPaintingNode : public rclcpp::Node
{
public:
  explicit PointPaintingNode(const rclcpp::NodeOptions & node_options);

private:
  void fusionCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi0_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi1_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi2_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi3_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi4_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi5_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi6_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi7_msg);

  void cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_camera_info_msg, const int id);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> v_camera_info_sub_;
  std::vector<std::shared_ptr<
    message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature>>>
    v_roi_sub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr painted_pointcloud_pub_;

  message_filters::PassThrough<tier4_perception_msgs::msg::DetectedObjectsWithFeature> passthrough_;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_ptr_;
  inline void dummyCallback(
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input)
  {
    auto dummy = input;
    passthrough_.add(dummy);
  }
  static uint8_t getSemanticType(const std::string & class_name);
  static bool isCarLikeVehicleLabel(const uint8_t label);

  int rois_number_;
  std::map<int, sensor_msgs::msg::CameraInfo> m_camera_info_;
  // std::shared_ptr<Debugger> debugger_;

  float score_threshold_{0.0};
  bool use_encoder_trt_{false};
  bool use_head_trt_{false};
  std::string trt_precision_;

  std::string encoder_onnx_path_;
  std::string encoder_engine_path_;
  std::string encoder_pt_path_;
  std::string head_onnx_path_;
  std::string head_engine_path_;
  std::string head_pt_path_;

  std::vector<std::string> class_names_;
  bool rename_car_to_truck_and_bus_{false};

  std::unique_ptr<CenterPointTRT> detector_ptr_{nullptr};
};

}  // namespace pointpainting

#endif  // POINTPAINTING__NODE_HPP_
