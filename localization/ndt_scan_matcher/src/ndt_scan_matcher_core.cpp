/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ndt_scan_matcher/ndt_scan_matcher_core.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <thread>

#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ndt_scan_matcher/util_func.h"

NDTScanMatcher::NDTScanMatcher(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh),
  private_nh_(private_nh),
  tf2_listener_(tf2_buffer_),
  ndt_implement_type_(NDTImplementType::PCL_GENERIC),
  base_frame_("base_link"),
  ndt_base_frame_("ndt_base_link"),
  map_frame_("map"),
  converged_param_transform_probability_(4.5)
{
  key_value_stdmap_["state"] = "Initializing";

  int ndt_implement_type_tmp = 0;
  private_nh_.getParam("ndt_implement_type", ndt_implement_type_tmp);
  ndt_implement_type_ = static_cast<NDTImplementType>(ndt_implement_type_tmp);
  if (ndt_implement_type_ == NDTImplementType::PCL_GENERIC) {
    ROS_INFO("NDT Implement Type is PCL GENERIC");
    ndt_ptr_.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
  } else if (ndt_implement_type_ == NDTImplementType::PCL_MODIFIED) {
    ROS_INFO("NDT Implement Type is PCL MODIFIED");
    ndt_ptr_.reset(new NormalDistributionsTransformPCLModified<PointSource, PointTarget>);
  } else if (ndt_implement_type_ == NDTImplementType::OMP) {
    ROS_INFO("NDT Implement Type is OMP");

    std::shared_ptr<NormalDistributionsTransformOMP<PointSource, PointTarget>> ndt_omp_ptr(
      new NormalDistributionsTransformOMP<PointSource, PointTarget>);

    int search_method = static_cast<int>(omp_params_.search_method);
    private_nh_.getParam("omp_neighborhood_search_method", search_method);
    omp_params_.search_method = static_cast<ndt_omp::NeighborSearchMethod>(search_method);
    // TODO check search_method is valid value.
    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);

    private_nh_.getParam("omp_num_threads", omp_params_.num_threads);
    omp_params_.num_threads = std::max(omp_params_.num_threads, 1);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);

    ndt_ptr_ = ndt_omp_ptr;
  } else {
    ndt_implement_type_ = NDTImplementType::PCL_GENERIC;
    ROS_INFO("NDT Implement Type is PCL GENERIC");
    ndt_ptr_.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
  }

  int points_queue_size = 0;
  private_nh_.getParam("input_sensor_points_queue_size", points_queue_size);
  points_queue_size = std::max(points_queue_size, 0);
  ROS_INFO("points_queue_size: %d", points_queue_size);

  private_nh_.getParam("base_frame", base_frame_);
  ROS_INFO("base_frame_id: %s", base_frame_.c_str());

  double trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  double step_size = ndt_ptr_->getStepSize();
  double resolution = ndt_ptr_->getResolution();
  int max_iterations = ndt_ptr_->getMaximumIterations();
  private_nh_.getParam("trans_epsilon", trans_epsilon);
  private_nh_.getParam("step_size", step_size);
  private_nh_.getParam("resolution", resolution);
  private_nh_.getParam("max_iterations", max_iterations);
  ndt_ptr_->setTransformationEpsilon(trans_epsilon);
  ndt_ptr_->setStepSize(step_size);
  ndt_ptr_->setResolution(resolution);
  ndt_ptr_->setMaximumIterations(max_iterations);
  ROS_INFO(
    "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon,
    step_size, resolution, max_iterations);

  private_nh_.getParam(
    "converged_param_transform_probability", converged_param_transform_probability_);

  initial_pose_sub_ =
    nh_.subscribe("ekf_pose_with_covariance", 100, &NDTScanMatcher::callbackInitialPose, this);
  map_points_sub_ = nh_.subscribe("pointcloud_map", 1, &NDTScanMatcher::callbackMapPoints, this);
  sensor_points_sub_ = nh_.subscribe("points_raw", 1, &NDTScanMatcher::callbackSensorPoints, this);

  sensor_aligned_pose_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);
  ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ndt_pose_with_covariance", 10);
  initial_pose_with_covariance_pub_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initial_pose_with_covariance", 10);
  exe_time_pub_ = nh_.advertise<std_msgs::Float32>("exe_time_ms", 10);
  transform_probability_pub_ = nh_.advertise<std_msgs::Float32>("transform_probability", 10);
  iteration_num_pub_ = nh_.advertise<std_msgs::Float32>("iteration_num", 10);
  initial_to_result_distance_pub_ =
    nh_.advertise<std_msgs::Float32>("initial_to_result_distance", 10);
  initial_to_result_distance_old_pub_ =
    nh_.advertise<std_msgs::Float32>("initial_to_result_distance_old", 10);
  initial_to_result_distance_new_pub_ =
    nh_.advertise<std_msgs::Float32>("initial_to_result_distance_new", 10);
  ndt_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("ndt_marker", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    nh_.advertise<visualization_msgs::MarkerArray>("monte_carlo_initial_pose_marker", 10);
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);

  service_ = nh_.advertiseService("ndt_align_srv", &NDTScanMatcher::serviceNDTAlign, this);
  // setup dynamic reconfigure server
  // f_ = boost::bind(&NDTScanMatcher::configCallback, this, _1, _2);
  // server_.setCallback(f_);

  diagnostic_thread_ = std::thread(&NDTScanMatcher::timerDiagnostic, this);
  diagnostic_thread_.detach();
}

NDTScanMatcher::~NDTScanMatcher() {}

void NDTScanMatcher::timerDiagnostic()
{
  ros::Rate rate(100);
  while (ros::ok()) {
    diagnostic_msgs::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : key_value_stdmap_) {
      diagnostic_msgs::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing") {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1) {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5) {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }

    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.stamp = ros::Time::now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_.publish(diag_msg);

    rate.sleep();
  }
}

bool NDTScanMatcher::serviceNDTAlign(
  autoware_localization_srvs::PoseWithCovarianceStamped::Request & req,
  autoware_localization_srvs::PoseWithCovarianceStamped::Response & res)
{
  // get TF from pose_frame to map_frame
  geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
  getTransform(map_frame_, req.pose_with_cov.header.frame_id, TF_pose_to_map_ptr);

  // transform pose_frame to map_frame
  geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_initial_pose_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  tf2::doTransform(req.pose_with_cov, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);

  if (ndt_ptr_->getInputTarget() == nullptr) {
    // TODO wait for map pointcloud
    return false;
  }

  if (ndt_ptr_->getInputSource() == nullptr) {
    // TODO wait for sensor pointcloud
    return false;
  }

  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  key_value_stdmap_["state"] = "Aligning";
  res.pose_with_cov = alignUsingMonteCarlo(ndt_ptr_, *mapTF_initial_pose_msg_ptr);
  key_value_stdmap_["state"] = "Sleeping";
  res.pose_with_cov.pose.covariance = req.pose_with_cov.pose.covariance;

  return true;
}

void NDTScanMatcher::callbackInitialPose(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & initial_pose_msg_ptr)
{
  // if rosbag restart, clear buffer
  if (!initial_pose_msg_ptr_array_.empty()) {
    if (initial_pose_msg_ptr_array_.front()->header.stamp > initial_pose_msg_ptr->header.stamp) {
      initial_pose_msg_ptr_array_.clear();
    }
  }

  if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
    initial_pose_msg_ptr_array_.push_back(initial_pose_msg_ptr);
  } else {
    // get TF from pose_frame to map_frame
    geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
    getTransform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    // transform pose_frame to map_frame
    geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_initial_pose_msg_ptr(
      new geometry_msgs::PoseWithCovarianceStamped);
    tf2::doTransform(*initial_pose_msg_ptr, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
    initial_pose_msg_ptr_array_.push_back(mapTF_initial_pose_msg_ptr);
  }
}

void NDTScanMatcher::callbackMapPoints(
  const sensor_msgs::PointCloud2::ConstPtr & map_points_msg_ptr)
{
  const auto trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  const auto step_size = ndt_ptr_->getStepSize();
  const auto resolution = ndt_ptr_->getResolution();
  const auto max_iterations = ndt_ptr_->getMaximumIterations();

  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> new_ndt_ptr_;

  if (ndt_implement_type_ == NDTImplementType::PCL_GENERIC) {
    new_ndt_ptr_.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
  } else if (ndt_implement_type_ == NDTImplementType::PCL_MODIFIED) {
    new_ndt_ptr_.reset(new NormalDistributionsTransformPCLModified<PointSource, PointTarget>);
  } else if (ndt_implement_type_ == NDTImplementType::OMP) {
    std::shared_ptr<NormalDistributionsTransformOMP<PointSource, PointTarget>> ndt_omp_ptr(
      new NormalDistributionsTransformOMP<PointSource, PointTarget>);

    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);

    new_ndt_ptr_ = ndt_omp_ptr;
  } else {
    new_ndt_ptr_.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
  }

  new_ndt_ptr_->setTransformationEpsilon(trans_epsilon);
  new_ndt_ptr_->setStepSize(step_size);
  new_ndt_ptr_->setResolution(resolution);
  new_ndt_ptr_->setMaximumIterations(max_iterations);

  pcl::PointCloud<PointTarget>::Ptr map_points_ptr(new pcl::PointCloud<PointTarget>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  new_ndt_ptr_->setInputTarget(map_points_ptr);
  // create Thread
  // detach
  pcl::PointCloud<PointSource>::Ptr output_cloud(new pcl::PointCloud<PointSource>);
  new_ndt_ptr_->align(*output_cloud, Eigen::Matrix4f::Identity());

  // swap
  ndt_map_mtx_.lock();
  ndt_ptr_ = new_ndt_ptr_;
  ndt_map_mtx_.unlock();
}

void NDTScanMatcher::callbackSensorPoints(
  const sensor_msgs::PointCloud2::ConstPtr & sensor_points_sensorTF_msg_ptr)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
  const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

  boost::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_sensorTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
  // get TF base to sensor
  geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
  getTransform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);
  const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();
  boost::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_baselinkTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::transformPointCloud(
    *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);
  ndt_ptr_->setInputSource(sensor_points_baselinkTF_ptr);

  // check
  if (initial_pose_msg_ptr_array_.empty()) {
    ROS_WARN_STREAM_THROTTLE(1, "No Pose!");
    return;
  }
  // searchNNPose using timestamp
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr initial_pose_old_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr initial_pose_new_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  getNearestTimeStampPose(
    initial_pose_msg_ptr_array_, sensor_ros_time, initial_pose_old_msg_ptr,
    initial_pose_new_msg_ptr);
  popOldPose(initial_pose_msg_ptr_array_, sensor_ros_time);
  // TODO check pose_timestamp - sensor_ros_time
  const auto initial_pose_msg =
    interpolatePose(*initial_pose_old_msg_ptr, *initial_pose_new_msg_ptr, sensor_ros_time);

  geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg;
  initial_pose_cov_msg.header = initial_pose_msg.header;
  initial_pose_cov_msg.pose.pose = initial_pose_msg.pose;

  if (ndt_ptr_->getInputTarget() == nullptr) {
    ROS_WARN_STREAM_THROTTLE(1, "No MAP!");
    return;
  }
  // align
  Eigen::Affine3d initial_pose_affine;
  tf2::fromMsg(initial_pose_cov_msg.pose.pose, initial_pose_affine);
  const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

  pcl::PointCloud<PointSource>::Ptr output_cloud(new pcl::PointCloud<PointSource>);
  const auto align_start_time = std::chrono::system_clock::now();
  key_value_stdmap_["state"] = "Aligning";
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);
  key_value_stdmap_["state"] = "Sleeping";
  const auto align_end_time = std::chrono::system_clock::now();
  const double align_time =
    std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time)
      .count() /
    1000.0;

  const Eigen::Matrix4f result_pose_matrix = ndt_ptr_->getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  const std::vector<Eigen::Matrix4f> result_pose_matrix_array =
    ndt_ptr_->getFinalTransformationArray();
  std::vector<geometry_msgs::Pose> result_pose_msg_array;
  for (const auto & pose_matrix : result_pose_matrix_array) {
    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose_matrix.cast<double>();
    const geometry_msgs::Pose pose_msg = tf2::toMsg(pose_affine);
    result_pose_msg_array.push_back(pose_msg);
  }

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() /
    1000.0;

  const float transform_probability = ndt_ptr_->getTransformationProbability();

  const size_t iteration_num = ndt_ptr_->getFinalNumIteration();

  bool is_converged = true;
  static size_t skipping_publish_num = 0;
  if (
    iteration_num >= ndt_ptr_->getMaximumIterations() + 2 ||
    transform_probability < converged_param_transform_probability_) {
    is_converged = false;
    ++skipping_publish_num;
    std::cout << "Not Converged" << std::endl;
  } else {
    skipping_publish_num = 0;
  }

  // publish
  geometry_msgs::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  //TODO temporary value
  result_pose_with_cov_msg.pose.covariance[0] = 0.025;
  result_pose_with_cov_msg.pose.covariance[1 * 6 + 1] = 0.025;
  result_pose_with_cov_msg.pose.covariance[2 * 6 + 2] = 0.025;
  result_pose_with_cov_msg.pose.covariance[3 * 6 + 3] = 0.000625;
  result_pose_with_cov_msg.pose.covariance[4 * 6 + 4] = 0.000625;
  result_pose_with_cov_msg.pose.covariance[5 * 6 + 5] = 0.000625;

  if (is_converged) {
    ndt_pose_pub_.publish(result_pose_stamped_msg);
    ndt_pose_with_covariance_pub_.publish(result_pose_with_cov_msg);
  }

  publishTF(map_frame_, ndt_base_frame_, result_pose_stamped_msg);

  pcl::PointCloud<PointSource>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<PointSource>);
  pcl::transformPointCloud(
    *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
  sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = map_frame_;
  sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);

  initial_pose_with_covariance_pub_.publish(initial_pose_cov_msg);

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.stamp = sensor_ros_time;
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::Marker::ADD;
  for (const auto & pose_msg : result_pose_msg_array) {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = ExchangeColorCrc((1.0 * i) / 15.0);
    marker_array.markers.push_back(marker);
  }
  // TODO delete old marker
  for (; i < ndt_ptr_->getMaximumIterations() + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::Pose();
    marker.color = ExchangeColorCrc(0);
    marker_array.markers.push_back(marker);
  }
  ndt_marker_pub_.publish(marker_array);

  std_msgs::Float32 exe_time_msg;
  exe_time_msg.data = exe_time;
  exe_time_pub_.publish(exe_time_msg);

  std_msgs::Float32 transform_probability_msg;
  transform_probability_msg.data = transform_probability;
  transform_probability_pub_.publish(transform_probability_msg);

  std_msgs::Float32 iteration_num_msg;
  iteration_num_msg.data = iteration_num;
  iteration_num_pub_.publish(iteration_num_msg);

  std_msgs::Float32 initial_to_result_distance_msg;
  initial_to_result_distance_msg.data = std::sqrt(
    std::pow(
      initial_pose_cov_msg.pose.pose.position.x - result_pose_with_cov_msg.pose.pose.position.x,
      2.0) +
    std::pow(
      initial_pose_cov_msg.pose.pose.position.y - result_pose_with_cov_msg.pose.pose.position.y,
      2.0) +
    std::pow(
      initial_pose_cov_msg.pose.pose.position.z - result_pose_with_cov_msg.pose.pose.position.z,
      2.0));
  initial_to_result_distance_pub_.publish(initial_to_result_distance_msg);

  std_msgs::Float32 initial_to_result_distance_old_msg;
  initial_to_result_distance_old_msg.data = std::sqrt(
    std::pow(
      initial_pose_old_msg_ptr->pose.pose.position.x -
        result_pose_with_cov_msg.pose.pose.position.x,
      2.0) +
    std::pow(
      initial_pose_old_msg_ptr->pose.pose.position.y -
        result_pose_with_cov_msg.pose.pose.position.y,
      2.0) +
    std::pow(
      initial_pose_old_msg_ptr->pose.pose.position.z -
        result_pose_with_cov_msg.pose.pose.position.z,
      2.0));
  initial_to_result_distance_old_pub_.publish(initial_to_result_distance_old_msg);

  std_msgs::Float32 initial_to_result_distance_new_msg;
  initial_to_result_distance_new_msg.data = std::sqrt(
    std::pow(
      initial_pose_new_msg_ptr->pose.pose.position.x -
        result_pose_with_cov_msg.pose.pose.position.x,
      2.0) +
    std::pow(
      initial_pose_new_msg_ptr->pose.pose.position.y -
        result_pose_with_cov_msg.pose.pose.position.y,
      2.0) +
    std::pow(
      initial_pose_new_msg_ptr->pose.pose.position.z -
        result_pose_with_cov_msg.pose.pose.position.z,
      2.0));
  initial_to_result_distance_new_pub_.publish(initial_to_result_distance_new_msg);

  key_value_stdmap_["seq"] = std::to_string(sensor_points_sensorTF_msg_ptr->header.seq);
  key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
  key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "align_time: " << align_time << "ms" << std::endl;
  std::cout << "exe_time: " << exe_time << "ms" << std::endl;
  std::cout << "trans_prob: " << transform_probability << std::endl;
  std::cout << "iter_num: " << iteration_num << std::endl;
  std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;
}

geometry_msgs::PoseWithCovarianceStamped NDTScanMatcher::alignUsingMonteCarlo(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
  const geometry_msgs::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    ROS_WARN("No Map or Sensor PointCloud");
    return geometry_msgs::PoseWithCovarianceStamped();
  }

  // generateParticle
  const auto initial_pose_array = createRandomPoseArray(initial_pose_with_cov, 100);

  std::vector<Particle> particle_array;
  pcl::PointCloud<PointSource>::Ptr output_cloud(new pcl::PointCloud<PointSource>);

  int i = 0;
  for (const auto & initial_pose : initial_pose_array.poses) {
    Eigen::Affine3d initial_pose_affine;
    tf2::fromMsg(initial_pose, initial_pose_affine);
    const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

    ndt_ptr->align(*output_cloud, initial_pose_matrix);

    const Eigen::Matrix4f result_pose_matrix = ndt_ptr->getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::Pose result_pose = tf2::toMsg(result_pose_affine);

    const auto transform_probability = ndt_ptr->getTransformationProbability();
    const auto num_iteration = ndt_ptr->getFinalNumIteration();

    Particle particle(initial_pose, result_pose, transform_probability, num_iteration);
    particle_array.push_back(particle);
    publishMarkerForDebug(particle, i++);

    pcl::PointCloud<PointSource>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<PointSource>);
    const auto sensor_points_baselinkTF_ptr = ndt_ptr->getInputSource();
    pcl::transformPointCloud(
      *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
    sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
    pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
    sensor_points_mapTF_msg.header.stamp = initial_pose_with_cov.header.stamp;
    sensor_points_mapTF_msg.header.frame_id = map_frame_;
    sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });
  // std::cout << "best score" << best_particle_ptr->score << std::endl;

  geometry_msgs::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;
  // ndt_pose_with_covariance_pub_.publish(result_pose_with_cov_msg);

  return result_pose_with_cov_msg;
}

void NDTScanMatcher::publishMarkerForDebug(const Particle & particle, const size_t i)
{
  // TODO getNumSubscribers
  // TODO clear old object
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.id = i;

  marker.ns = "initial_pose_transform_probability_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = ExchangeColorCrc(particle.score / 4.5);
  marker_array.markers.push_back(marker);

  marker.ns = "initial_pose_iteration_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = ExchangeColorCrc((1.0 * particle.iteration) / 30.0);
  marker_array.markers.push_back(marker);

  marker.ns = "initial_pose_index_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = ExchangeColorCrc((1.0 * i) / 100);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_transform_probability_color_marker";
  marker.pose = particle.result_pose;
  marker.color = ExchangeColorCrc(particle.score / 4.5);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_iteration_color_marker";
  marker.pose = particle.result_pose;
  marker.color = ExchangeColorCrc((1.0 * particle.iteration) / 30.0);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_index_color_marker";
  marker.pose = particle.result_pose;
  marker.color = ExchangeColorCrc((1.0 * i) / 100);
  marker_array.markers.push_back(marker);

  ndt_monte_carlo_initial_pose_marker_pub_.publish(marker_array);
}

void NDTScanMatcher::publishTF(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::PoseStamped & pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}

bool NDTScanMatcher::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr, const ros::Time & time_stamp)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp, ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool NDTScanMatcher::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}
