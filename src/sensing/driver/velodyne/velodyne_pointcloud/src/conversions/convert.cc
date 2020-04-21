/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include <velodyne_pointcloud/convert.h>

#include <tf/tf.h>

#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/pointcloudXYZIRADT.h>

#include <yaml-cpp/yaml.h>

#include <velodyne_pointcloud/func.h>

namespace velodyne_pointcloud
{
/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh)
: tf2_listener_(tf2_buffer_),
  data_(new velodyne_rawdata::RawData()),
  num_points_threshold_(300),
  base_link_frame_("base_link")
{
  ROS_WARN("This node is debugged only VLP16 and VLP32C. Please dont use other models");

  data_->setup(private_nh);

  private_nh.getParam("num_points_threshold", num_points_threshold_);

  std::string invalid_intensity;
  private_nh.getParam("invalid_intensity", invalid_intensity);
  YAML::Node invalid_intensity_yaml = YAML::Load(invalid_intensity);
  invalid_intensity_array_ = std::vector<float>(data_->getNumLasers(), 0);
  for (size_t i = 0; i < invalid_intensity_yaml.size(); ++i) {
    invalid_intensity_array_.at(i) = invalid_intensity_yaml[i].as<float>();
  }

  // advertise
  velodyne_points_pub_ = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
  velodyne_points_ex_pub_ = node.advertise<sensor_msgs::PointCloud2>("velodyne_points_ex", 10);
  velodyne_points_invalid_near_pub_ =
    node.advertise<sensor_msgs::PointCloud2>("velodyne_points_invalid_near", 10);
  velodyne_points_combined_ex_pub_ =
    node.advertise<sensor_msgs::PointCloud2>("velodyne_points_combined_ex", 10);
  marker_array_pub_ = node.advertise<visualization_msgs::MarkerArray>("velodyne_model_marker", 1);
  srv_ = boost::make_shared<dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig> >(
    private_nh);
  dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  // subscribe
  velodyne_scan_ = node.subscribe(
    "velodyne_packets", 10, &Convert::processScan, (Convert *)this,
    ros::TransportHints().tcpNoDelay(true));
}

void Convert::callback(velodyne_pointcloud::CloudNodeConfig & config, uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  data_->setParameters(
    config.min_range, config.max_range, config.view_direction, config.view_width);
  num_points_threshold_ = config.num_points_threshold;

  YAML::Node invalid_intensity_yaml = YAML::Load(config.invalid_intensity);
  invalid_intensity_array_ = std::vector<float>(data_->getNumLasers(), 0);
  for (size_t i = 0; i < invalid_intensity_yaml.size(); ++i) {
    invalid_intensity_array_.at(i) = invalid_intensity_yaml[i].as<float>();
  }
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr & scanMsg)
{
  velodyne_pointcloud::PointcloudXYZIRADT scan_points_xyziradt;
  if (
    velodyne_points_pub_.getNumSubscribers() > 0 ||
    velodyne_points_ex_pub_.getNumSubscribers() > 0 ||
    velodyne_points_invalid_near_pub_.getNumSubscribers() > 0 ||
    velodyne_points_combined_ex_pub_.getNumSubscribers() > 0) {
    scan_points_xyziradt.pc->points.reserve(scanMsg->packets.size() * data_->scansPerPacket());
    for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
      data_->unpack(scanMsg->packets[i], scan_points_xyziradt);
    }
    scan_points_xyziradt.pc->header = pcl_conversions::toPCL(scanMsg->header);
    scan_points_xyziradt.pc->header.stamp =
      pcl_conversions::toPCL(scanMsg->packets[0].stamp - ros::Duration(0.0));
    scan_points_xyziradt.pc->height = 1;
    scan_points_xyziradt.pc->width = scan_points_xyziradt.pc->points.size();
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr valid_points_xyziradt(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  if (
    velodyne_points_pub_.getNumSubscribers() > 0 ||
    velodyne_points_ex_pub_.getNumSubscribers() > 0 ||
    velodyne_points_combined_ex_pub_.getNumSubscribers() > 0) {
    valid_points_xyziradt =
      extractValidPoints(scan_points_xyziradt.pc, data_->getMinRange(), data_->getMaxRange());
    if (velodyne_points_pub_.getNumSubscribers() > 0) {
      const auto valid_points_xyzir = convert(valid_points_xyziradt);
      velodyne_points_pub_.publish(valid_points_xyzir);
    }
    if (velodyne_points_ex_pub_.getNumSubscribers() > 0) {
      velodyne_points_ex_pub_.publish(valid_points_xyziradt);
    }
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr invalid_near_points_filtered_xyziradt(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  if (
    velodyne_points_invalid_near_pub_.getNumSubscribers() > 0 ||
    velodyne_points_combined_ex_pub_.getNumSubscribers() > 0) {
    const size_t num_lasers = data_->getNumLasers();
    const auto sorted_invalid_points_xyziradt = sortZeroIndex(scan_points_xyziradt.pc, num_lasers);
    invalid_near_points_filtered_xyziradt = extractInvalidNearPointsFiltered(
      sorted_invalid_points_xyziradt, invalid_intensity_array_, num_lasers, num_points_threshold_);
    if (velodyne_points_invalid_near_pub_.getNumSubscribers() > 0) {
      const auto invalid_near_points_filtered_xyzir =
        convert(invalid_near_points_filtered_xyziradt);
      velodyne_points_invalid_near_pub_.publish(invalid_near_points_filtered_xyzir);
    }
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr combined_points_xyziradt(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  if (velodyne_points_combined_ex_pub_.getNumSubscribers() > 0) {
    combined_points_xyziradt->points.reserve(
      valid_points_xyziradt->points.size() + invalid_near_points_filtered_xyziradt->points.size());
    combined_points_xyziradt->points.insert(
      std::end(combined_points_xyziradt->points), std::begin(valid_points_xyziradt->points),
      std::end(valid_points_xyziradt->points));
    combined_points_xyziradt->points.insert(
      std::end(combined_points_xyziradt->points),
      std::begin(invalid_near_points_filtered_xyziradt->points),
      std::end(invalid_near_points_filtered_xyziradt->points));
    combined_points_xyziradt->header = pcl_conversions::toPCL(scanMsg->header);
    combined_points_xyziradt->height = 1;
    combined_points_xyziradt->width = combined_points_xyziradt->points.size();
    velodyne_points_combined_ex_pub_.publish(combined_points_xyziradt);
  }

  if (marker_array_pub_.getNumSubscribers() > 0) {
    const auto velodyne_model_marker = createVelodyneModelMakerMsg(scanMsg->header);
    marker_array_pub_.publish(velodyne_model_marker);
  }
}

visualization_msgs::MarkerArray Convert::createVelodyneModelMakerMsg(
  const std_msgs::Header & header)
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  auto generateQuaternion = [](double roll, double pitch, double yaw) {
    geometry_msgs::Quaternion quta;
    quaternionTFToMsg(tf::createQuaternionFromRPY(roll, pitch, yaw), quta);
    return quta;
  };

  auto generateVector3 = [](double x, double y, double z) {
    geometry_msgs::Vector3 vec;
    vec.x = x;
    vec.y = y;
    vec.z = z;
    return vec;
  };

  auto generateColor = [](float r, float g, float b, float a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  };

  //array[0]:bottom body, array[1]:middle body(laser window), array[2]: top body, array[3]:cable
  const double radius = 0.1033;
  const std::array<geometry_msgs::Point, 4> pos = {
    generatePoint(0.0, 0.0, -0.0285), generatePoint(0.0, 0.0, 0.0), generatePoint(0.0, 0.0, 0.0255),
    generatePoint(-radius / 2.0 - 0.005, 0.0, -0.03)};
  const std::array<geometry_msgs::Quaternion, 4> quta = {
    generateQuaternion(0.0, 0.0, 0.0), generateQuaternion(0.0, 0.0, 0.0),
    generateQuaternion(0.0, 0.0, 0.0), generateQuaternion(0.0, M_PI_2, 0.0)};
  const std::array<geometry_msgs::Vector3, 4> scale = {
    generateVector3(radius, radius, 0.020), generateVector3(radius, radius, 0.037),
    generateVector3(radius, radius, 0.015), generateVector3(0.0127, 0.0127, 0.02)};
  const std::array<std_msgs::ColorRGBA, 4> color = {
    generateColor(0.85, 0.85, 0.85, 0.85), generateColor(0.1, 0.1, 0.1, 0.98),
    generateColor(0.85, 0.85, 0.85, 0.85), generateColor(0.2, 0.2, 0.2, 0.98)};

  visualization_msgs::MarkerArray marker_array_msg;
  for (size_t i = 0; i < 4; ++i) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = std::string(header.frame_id) + "_velodyne_model";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = pos[i];
    marker.pose.orientation = quta[i];
    marker.scale = scale[i];
    marker.color = color[i];
    marker_array_msg.markers.push_back(marker);
  }

  return marker_array_msg;
}

bool Convert::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  tf2::Transform * tf2_transform_ptr)
{
  if (target_frame == source_frame) {
    tf2_transform_ptr->setOrigin(tf2::Vector3(0, 0, 0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0, 0, 0, 1));
    return true;
  }

  try {
    const auto transform_msg =
      tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    tf2::convert(transform_msg.transform, *tf2_transform_ptr);
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    tf2_transform_ptr->setOrigin(tf2::Vector3(0, 0, 0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0, 0, 0, 1));
    return false;
  }
  return true;
}

}  // namespace velodyne_pointcloud
