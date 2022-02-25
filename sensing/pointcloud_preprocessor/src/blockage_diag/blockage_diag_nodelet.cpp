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

#include "pointcloud_preprocessor/blockage_diag/blockage_diag_nodelet.hpp"

#include <std_msgs/msg/header.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <algorithm>
#include <string>
#include <vector>
#include <boost/thread/detail/platform_time.hpp>



namespace pointcloud_preprocessor
{
using diagnostic_msgs::msg::DiagnosticStatus;

BlockageDiagComponent::BlockageDiagComponent(
  const rclcpp::NodeOptions & options)
: Filter("BlockageDiag",options)
{
  {
    //initialize params:
    ground_ring_id_ = static_cast<uint>(declare_parameter("ground_ring_id",12));
    ground_blockage_threshold_ = static_cast<float>(declare_parameter("ground_blockage_threshold",0.1));
    sky_blockage_threshold_ = static_cast<float>(declare_parameter("sky_blockage_threshold",0.2));
    erode_iterator_ = static_cast<uint>(declare_parameter("erode_iterator",5));
    erode_kernel_ = static_cast<uint>(declare_parameter("erode_kernel",3));
    vertical_bins_ = static_cast<uint>(declare_parameter("vertical_bins",64));
    resolution_ = static_cast<float>(declare_parameter("resolution",100.0));
    angle_range_ = declare_parameter("angle_range",std::vector<double>{0.0,360.0});
    distance_range_ = declare_parameter("distance_range",std::vector<double>{0.1,200.0});
    lidar_model_ = static_cast<std::string>(declare_parameter("model","Pandar40P"));

  }

  updater_.setHardwareID("ground_blockage_diag");
  updater_.add(std::string(this->get_namespace())+": ground_blockage_validation", this, &BlockageDiagComponent::onBlockageChecker);
  updater_.setPeriod(0.1);

  updater_.setHardwareID("sky_blockage_diag");
  updater_.add(std::string(this->get_namespace()) + ": sky_blockage_validation", this, &BlockageDiagComponent::onSkyBlockageChecker);
  updater_.setPeriod(0.1);


  lidar_depth_map_pub_ = 
    image_transport::create_publisher(this,"blockage_diag/debug/lidar_depth_map");
  blockage_mask_pub_ = image_transport::create_publisher(this,"blockage_diag/debug/blockage_mask_image");

  blockage_ratio_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "blockage_diag/debug/blockage_ratio",rclcpp::SensorDataQoS()
  );
    blockage_range_pub_ = create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>(
    "blockage_diag/debug/blockage_range",rclcpp::SensorDataQoS()
  );

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&BlockageDiagComponent::paramCallback,this,_1));
}

void BlockageDiagComponent::onBlockageChecker(DiagnosticStatusWrapper & stat){
  stat.add("ground_range_blockage_ratio", std::to_string(ground_blockage_ratio_));
  stat.add("blockage_count", std::to_string(ground_blockage_count_));
  stat.add("blockage_range_x1", std::to_string(ground_blockage_boundingbox_[0]));
  stat.add("blockage_range_y1", std::to_string(ground_blockage_boundingbox_[1]));
  stat.add("blockage_range_x2", std::to_string(ground_blockage_boundingbox_[2]));
  stat.add("blockage_range_y2", std::to_string(ground_blockage_boundingbox_[3]));

  auto level = DiagnosticStatus::OK;
  if (ground_blockage_ratio_ < 0){
    level = DiagnosticStatus::STALE;
  }else if (ground_blockage_ratio_ > ground_blockage_threshold_){
    level = DiagnosticStatus::ERROR;
  }else if (sky_blockage_ratio_ > ground_blockage_threshold_){
    level = DiagnosticStatus::WARN;
  }else {
    level = DiagnosticStatus::OK;
  }

  std::string msg;
  if (level == DiagnosticStatus::OK){
    msg = "OK";
  }else if (level == DiagnosticStatus::WARN){
    msg = "WARNING: LiDAR blockage";
  }else if (level == DiagnosticStatus::ERROR){
    msg = "ERROR: LiDAR blockage";
  }else if (level == DiagnosticStatus::STALE){
    msg = "STALE";
  }
  stat.summary(level, msg);
}

void BlockageDiagComponent::onSkyBlockageChecker(DiagnosticStatusWrapper & stat){
  stat.add("sky_range_blockage_ratio", std::to_string(sky_blockage_ratio_));
  stat.add("blockage_count", std::to_string(sky_blockage_count_));
  stat.add("blockage_range_x1", std::to_string(sky_blockage_boundingbox_[0]));
  stat.add("blockage_range_y1", std::to_string(sky_blockage_boundingbox_[1]));
  stat.add("blockage_range_x2", std::to_string(sky_blockage_boundingbox_[2]));
  stat.add("blockage_range_y2", std::to_string(sky_blockage_boundingbox_[3]));

  auto level = DiagnosticStatus::OK;
  if (ground_blockage_ratio_ < 0){
    level = DiagnosticStatus::STALE;
  }else if (ground_blockage_ratio_ > ground_blockage_threshold_){
    level = DiagnosticStatus::ERROR;
  }else if (sky_blockage_ratio_ > ground_blockage_threshold_){
    level = DiagnosticStatus::WARN;
  }else {
    level = DiagnosticStatus::OK;
  }

  std::string msg;
  if (level == DiagnosticStatus::OK){
    msg = "OK";
  }else if (level == DiagnosticStatus::WARN){
    msg = "WARNING: LiDAR blockage";
  }else if (level == DiagnosticStatus::ERROR){
    msg = "ERROR: LiDAR blockage";
  }else if (level == DiagnosticStatus::STALE){
    msg = "STALE";
  }
  stat.summary(level, msg);
}


void BlockageDiagComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  bound_left_ = static_cast<float>(angle_range_[0])*100.0f;
  bound_right_ = static_cast<float>(angle_range_[1])*100.0f;
  max_distance_ = static_cast<float>(distance_range_[1]);
  uint horizontal_bins = static_cast<uint>((bound_right_ - bound_left_)/resolution_);
  uint vertical_bins = vertical_bins_;
  pcl::PointCloud<return_type_cloud::PointXYZIRADT>::Ptr pcl_input(
    new pcl::PointCloud<return_type_cloud::PointXYZIRADT>);
  pcl::fromROSMsg(*input, *pcl_input);
  cv::Mat lidar_depth_map(cv::Size(horizontal_bins,vertical_bins),CV_16UC1,cv::Scalar(0));
  cv::Mat lidar_depth_map_8u(cv::Size(horizontal_bins,vertical_bins),CV_8UC1,cv::Scalar(0));
  if (pcl_input->points.empty()){
    ground_blockage_ratio_ = 1.0f;
    sky_blockage_ratio_ = 1.0f;
    blockage_ratio_ = 1.0f;
    ground_blockage_count_ += 1;
    sky_blockage_count_ += 1;
    ground_blockage_boundingbox_[0] = bound_left_ / resolution_;
    ground_blockage_boundingbox_[1] = 0;
    ground_blockage_boundingbox_[2] = bound_right_ / resolution_;
    ground_blockage_boundingbox_[3] = vertical_bins;


    sky_blockage_boundingbox_[0] = bound_left_ / resolution_;
    sky_blockage_boundingbox_[1] = 0;
    sky_blockage_boundingbox_[2] = bound_right_ / resolution_;
    sky_blockage_boundingbox_[3] = vertical_bins;

  }
  else
  {
    for (const auto &p : pcl_input->points){
      if((p.azimuth > bound_left_) && (p.azimuth < bound_right_) && (p.return_type != ReturnType::DUAL_WEAK_FIRST)){
        if (lidar_model_ == "Pandar40P"){
        lidar_depth_map.at<uint16_t>(p.ring, static_cast<uint>((p.azimuth - bound_left_) / resolution_)) +=
          static_cast<uint16_t>(255.0f / p.distance * 50.0f); // make image clearly
        }
        else{
          lidar_depth_map.at<uint16_t>(vertical_bins - p.ring -1, static_cast<uint>((p.azimuth - bound_left_) / resolution_)) += 
            static_cast<uint16_t>(255.0f / p.distance *50.0f);
        }
      }
    }
    cv::Mat ground_lidar_depth_map(cv::Size(horizontal_bins, ground_ring_id_),CV_8UC1);
    cv::Mat sky_lidar_depth_map(cv::Size(horizontal_bins, vertical_bins - ground_ring_id_),CV_8UC1);  
    lidar_depth_map.convertTo(lidar_depth_map_8u, CV_8UC1, 1.0/resolution_);
    cv::Mat no_return_mask;
    cv::inRange(lidar_depth_map_8u, 0, 1, no_return_mask);
    cv::Mat erosion_dst;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                        cv::Size(2*erode_kernel_ + 1, 2 * erode_kernel_ + 1),
                        cv::Point(erode_kernel_, erode_kernel_));
    cv::erode(no_return_mask,erosion_dst,element);
    cv::dilate(erosion_dst, no_return_mask,element);
    cv::Mat ground_no_return_mask;
    cv::Mat sky_no_return_mask;
    no_return_mask(cv::Rect(0,0,horizontal_bins,ground_ring_id_)).copyTo(sky_no_return_mask);
    no_return_mask(cv::Rect(0,ground_ring_id_,horizontal_bins,vertical_bins - 
      ground_ring_id_)).copyTo(ground_no_return_mask);
    ground_blockage_ratio_ = static_cast<float>(cv::countNonZero(ground_no_return_mask)) / 
      static_cast<float>(horizontal_bins * (vertical_bins - ground_ring_id_));
    sky_blockage_ratio_ = static_cast<float>(cv::countNonZero(sky_no_return_mask)) / 
      static_cast<float>(horizontal_bins * (ground_ring_id_));

    if (ground_blockage_ratio_ > ground_blockage_threshold_){
      cv::Rect boundingbox = cv::boundingRect(ground_no_return_mask);
      ground_blockage_boundingbox_[0] = static_cast<float>(boundingbox.x) + bound_left_ / resolution_;
      ground_blockage_boundingbox_[1] = static_cast<float>(boundingbox.y) + ground_ring_id_;
      ground_blockage_boundingbox_[2] = static_cast<float>(boundingbox.x + boundingbox.width ) + bound_left_ / resolution_;
      ground_blockage_boundingbox_[3] = static_cast<float>(boundingbox.y + boundingbox.height) + ground_ring_id_;

      ground_blockage_count_ += 1;
    }
    else{
        ground_blockage_count_ = 0;
      }

    if (sky_blockage_ratio_ > sky_blockage_threshold_){
      cv::Rect sky_bx = cv::boundingRect(sky_no_return_mask);
      sky_blockage_boundingbox_[0] = static_cast<float>(sky_bx.x) + bound_left_ / resolution_;
      sky_blockage_boundingbox_[1] = static_cast<float>(sky_bx.y);
      sky_blockage_boundingbox_[2] = static_cast<float>(sky_bx.x + sky_bx.width ) + bound_left_ / resolution_;
      sky_blockage_boundingbox_[3] = static_cast<float>(sky_bx.y + sky_bx.height);

      sky_blockage_count_ += 1;
    }
    else{
        sky_blockage_count_ = 0;
      }
  
    cv::Mat lidar_depth_colorized;
    cv::applyColorMap(lidar_depth_map_8u, lidar_depth_colorized, cv::COLORMAP_JET);
    sensor_msgs::msg::Image::SharedPtr lidar_depth_msg = 
      cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",lidar_depth_colorized).toImageMsg();
    lidar_depth_msg->header = input->header;
    lidar_depth_map_pub_.publish(lidar_depth_msg);

    cv::Mat blockage_mask_colorized;
    cv::applyColorMap(no_return_mask, blockage_mask_colorized, cv::COLORMAP_JET);
    sensor_msgs::msg::Image::SharedPtr blockage_mask_msg = 
      cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8", blockage_mask_colorized).toImageMsg();
    blockage_mask_msg->header = input->header;
    blockage_mask_pub_.publish(blockage_mask_msg);

  }


  tier4_debug_msgs::msg::Float32Stamped blockage_ratio_msg;
  blockage_ratio_msg.data = blockage_ratio_;
  blockage_ratio_msg.stamp = now();
  blockage_ratio_pub_->publish(blockage_ratio_msg);
  
  tier4_debug_msgs::msg::Float32MultiArrayStamped blockage_azimuth_range_msg;
  blockage_azimuth_range_msg.data = ground_blockage_boundingbox_;
  blockage_azimuth_range_msg.stamp = now();
  blockage_range_pub_->publish(blockage_azimuth_range_msg);

  pcl::toROSMsg(*pcl_input,output);
  output.header = input->header;
  
}
rcl_interfaces::msg::SetParametersResult BlockageDiagComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  boost::mutex::scoped_lock lock(mutex_);
    if (get_param(p, "partial_blockage_threshold", ground_blockage_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new partial_blockage_threshold to: %f.", ground_blockage_threshold_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::BlockageDiagComponent)
