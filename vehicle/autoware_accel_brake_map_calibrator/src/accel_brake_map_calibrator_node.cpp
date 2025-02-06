//
// Copyright 2020 Tier IV, Inc. All rights reserved.
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
//

#include "autoware_accel_brake_map_calibrator/accel_brake_map_calibrator_node.hpp"

#include "rclcpp/logging.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace autoware::accel_brake_map_calibrator
{

AccelBrakeMapCalibrator::AccelBrakeMapCalibrator(const rclcpp::NodeOptions & node_options)
: Node("accel_brake_map_calibrator", node_options)
{
  transform_listener_ = std::make_shared<autoware::universe_utils::TransformListener>(this);
  // get parameter
  update_hz_ = declare_parameter<double>("update_hz", 10.0);
  covariance_ = declare_parameter<double>("initial_covariance", 0.05);
  velocity_min_threshold_ = declare_parameter<double>("velocity_min_threshold", 0.1);
  velocity_diff_threshold_ = declare_parameter<double>("velocity_diff_threshold", 0.556);
  pedal_diff_threshold_ = declare_parameter<double>("pedal_diff_threshold", 0.03);
  max_steer_threshold_ = declare_parameter<double>("max_steer_threshold", 0.2);
  max_pitch_threshold_ = declare_parameter<double>("max_pitch_threshold", 0.02);
  max_jerk_threshold_ = declare_parameter<double>("max_jerk_threshold", 0.7);
  pedal_velocity_thresh_ = declare_parameter<double>("pedal_velocity_thresh", 0.15);
  max_accel_ = declare_parameter<double>("max_accel", 5.0);
  min_accel_ = declare_parameter<double>("min_accel", -5.0);
  pedal_to_accel_delay_ = declare_parameter<double>("pedal_to_accel_delay", 0.3);
  max_data_count_ = static_cast<int>(declare_parameter("max_data_count", 200));
  pedal_accel_graph_output_ = declare_parameter<bool>("pedal_accel_graph_output", false);
  progress_file_output_ = declare_parameter<bool>("progress_file_output", false);
  precision_ = static_cast<int>(declare_parameter("precision", 3));
  const auto get_pitch_method_str = declare_parameter("get_pitch_method", std::string("tf"));
  if (get_pitch_method_str == std::string("tf")) {
    get_pitch_method_ = GET_PITCH_METHOD::TF;
  } else if (get_pitch_method_str == std::string("none")) {
    get_pitch_method_ = GET_PITCH_METHOD::NONE;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "update_method_ is wrong. (available method: tf, file, none");
    return;
  }
  const auto accel_brake_value_source_str =
    declare_parameter("accel_brake_value_source", std::string("status"));
  if (accel_brake_value_source_str == std::string("status")) {
    accel_brake_value_source_ = ACCEL_BRAKE_SOURCE::STATUS;
  } else if (accel_brake_value_source_str == std::string("command")) {
    accel_brake_value_source_ = ACCEL_BRAKE_SOURCE::COMMAND;
  } else {
    RCLCPP_ERROR_STREAM(
      get_logger(), "accel_brake_value_source is wrong. (available source: status, command");
    return;
  }

  update_suggest_thresh_ = declare_parameter<double>("update_suggest_thresh", 0.7);
  csv_calibrated_map_dir_ = declare_parameter("csv_calibrated_map_dir", std::string(""));
  output_accel_file_ = csv_calibrated_map_dir_ + "/accel_map.csv";
  output_brake_file_ = csv_calibrated_map_dir_ + "/brake_map.csv";
  const std::string update_method_str =
    declare_parameter("update_method", std::string("update_offset_each_cell"));
  if (update_method_str == std::string("update_offset_each_cell")) {
    update_method_ = UPDATE_METHOD::UPDATE_OFFSET_EACH_CELL;
  } else if (update_method_str == std::string("update_offset_total")) {
    update_method_ = UPDATE_METHOD::UPDATE_OFFSET_TOTAL;
  } else if (update_method_str == std::string("update_offset_four_cell_around")) {
    update_method_ = UPDATE_METHOD::UPDATE_OFFSET_FOUR_CELL_AROUND;
  } else {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "update_method is wrong. (available method: update_offset_each_cell, update_offset_total");
    return;
  }
  // initializer

  // QoS setup
  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);

  // Publisher for check_update_suggest
  calibration_status_pub_ = create_publisher<CalibrationStatus>(
    "/accel_brake_map_calibrator/output/calibration_status", durable_qos);

  /* Diagnostic Updater */
  updater_ptr_ = std::make_shared<diagnostic_updater::Updater>(this, 1.0 / update_hz_);
  updater_ptr_->setHardwareID("accel_brake_map_calibrator");
  updater_ptr_->add(
    "accel_brake_map_calibrator", this, &AccelBrakeMapCalibrator::check_update_suggest);

  {
    csv_default_map_dir_ = declare_parameter("csv_default_map_dir", std::string(""));

    std::string csv_path_accel_map = csv_default_map_dir_ + "/accel_map.csv";
    std::string csv_path_brake_map = csv_default_map_dir_ + "/brake_map.csv";
    if (
      !accel_map_.readAccelMapFromCSV(csv_path_accel_map) ||
      !new_accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
      is_default_map_ = false;
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Cannot read accelmap. csv path = " << csv_path_accel_map.c_str() << ". stop calculation.");
      return;
    }
    if (
      !brake_map_.readBrakeMapFromCSV(csv_path_brake_map) ||
      !new_brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
      is_default_map_ = false;
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Cannot read brakemap. csv path = " << csv_path_brake_map.c_str() << ". stop calculation.");
      return;
    }
  }

  std::string output_log_file = declare_parameter("output_log_file", std::string(""));
  output_log_.open(output_log_file);
  add_index_to_csv(&output_log_);

  debug_values_.data.resize(num_debug_values_);

  // input map info
  accel_map_value_ = accel_map_.getAccelMap();
  brake_map_value_ = brake_map_.getBrakeMap();
  accel_vel_index_ = accel_map_.getVelIdx();
  brake_vel_index_ = brake_map_.getVelIdx();
  accel_pedal_index_ = accel_map_.getThrottleIdx();
  brake_pedal_index_ = brake_map_.getBrakeIdx();
  update_accel_map_value_.resize((accel_map_value_.size()));
  update_brake_map_value_.resize((brake_map_value_.size()));
  for (auto & m : update_accel_map_value_) {
    m.resize(accel_map_value_.at(0).size());
  }
  for (auto & m : update_brake_map_value_) {
    m.resize(brake_map_value_.at(0).size());
  }
  accel_offset_covariance_value_.resize((accel_map_value_.size()));
  brake_offset_covariance_value_.resize((brake_map_value_.size()));
  for (auto & m : accel_offset_covariance_value_) {
    m.resize(accel_map_value_.at(0).size(), covariance_);
  }
  for (auto & m : brake_offset_covariance_value_) {
    m.resize(brake_map_value_.at(0).size(), covariance_);
  }
  map_value_data_.resize(accel_map_value_.size() + brake_map_value_.size() - 1);
  for (auto & m : map_value_data_) {
    m.resize(accel_map_value_.at(0).size());
  }

  std::copy(accel_map_value_.begin(), accel_map_value_.end(), update_accel_map_value_.begin());
  std::copy(brake_map_value_.begin(), brake_map_value_.end(), update_brake_map_value_.begin());

  // initialize matrix for covariance calculation
  {
    const auto gen_const_mat = [](const Map & map, const auto val) {
      return Eigen::MatrixXd::Constant(map.size(), map.at(0).size(), val);
    };
    accel_data_mean_mat_ = gen_const_mat(accel_map_value_, map_offset_);
    brake_data_mean_mat_ = gen_const_mat(brake_map_value_, map_offset_);
    accel_data_covariance_mat_ = gen_const_mat(accel_map_value_, covariance_);
    brake_data_covariance_mat_ = gen_const_mat(brake_map_value_, covariance_);
    accel_data_num_ = gen_const_mat(accel_map_value_, 1);
    brake_data_num_ = gen_const_mat(brake_map_value_, 1);
  }

  // publisher
  update_suggest_pub_ =
    create_publisher<std_msgs::msg::Bool>("~/output/update_suggest", durable_qos);
  original_map_occ_pub_ = create_publisher<OccupancyGrid>("~/debug/original_occ_map", durable_qos);
  update_map_occ_pub_ = create_publisher<OccupancyGrid>("~/debug/update_occ_map", durable_qos);
  data_ave_pub_ = create_publisher<OccupancyGrid>("~/debug/data_average_occ_map", durable_qos);
  data_std_pub_ = create_publisher<OccupancyGrid>("~/debug/data_std_dev_occ_map", durable_qos);
  data_count_pub_ = create_publisher<OccupancyGrid>("~/debug/data_count_occ_map", durable_qos);
  data_count_with_self_pose_pub_ =
    create_publisher<OccupancyGrid>("~/debug/data_count_self_pose_occ_map", durable_qos);
  index_pub_ = create_publisher<MarkerArray>("~/debug/occ_index", durable_qos);
  original_map_raw_pub_ =
    create_publisher<Float32MultiArray>("~/debug/original_raw_map", durable_qos);
  update_map_raw_pub_ = create_publisher<Float32MultiArray>("~/output/update_raw_map", durable_qos);
  debug_pub_ = create_publisher<Float32MultiArrayStamped>("~/output/debug_values", durable_qos);
  current_map_error_pub_ =
    create_publisher<Float32Stamped>("~/output/current_map_error", durable_qos);
  updated_map_error_pub_ =
    create_publisher<Float32Stamped>("~/output/updated_map_error", durable_qos);
  map_error_ratio_pub_ = create_publisher<Float32Stamped>("~/output/map_error_ratio", durable_qos);
  offset_covariance_pub_ =
    create_publisher<Float32MultiArray>("~/debug/offset_covariance", durable_qos);

  // subscriber
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  // Service
  update_map_dir_server_ = create_service<UpdateAccelBrakeMap>(
    "~/input/update_map_dir",
    std::bind(&AccelBrakeMapCalibrator::callback_update_map_service, this, _1, _2, _3));

  // timer
  init_timer(1.0 / update_hz_);
  init_output_csv_timer(30.0);

  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);
}

void AccelBrakeMapCalibrator::init_output_csv_timer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_output_csv_ = rclcpp::create_timer(
    this, get_clock(), period_ns,
    std::bind(&AccelBrakeMapCalibrator::timer_callback_output_csv, this));
}

void AccelBrakeMapCalibrator::init_timer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&AccelBrakeMapCalibrator::fetch_data, this));
}

bool AccelBrakeMapCalibrator::get_current_pitch_from_tf(double * pitch)
{
  if (get_pitch_method_ == GET_PITCH_METHOD::NONE) {
    // do not get pitch from tf
    *pitch = 0.0;
    return true;
  }

  // get tf
  const auto transform = transform_listener_->getTransform(
    "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
  if (!transform) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 5000, "cannot get map to base_link transform. ");
    return false;
  }
  double roll = 0.0;
  double raw_pitch = 0.0;
  double yaw = 0.0;
  tf2::getEulerYPR(transform->transform.rotation, roll, raw_pitch, yaw);
  debug_values_.data.at(CURRENT_RAW_PITCH) = static_cast<float>(raw_pitch);
  *pitch = lowpass(*pitch, raw_pitch, 0.2);
  debug_values_.data.at(CURRENT_PITCH) = static_cast<float>(*pitch);
  return true;
}

bool AccelBrakeMapCalibrator::take_data()
{
  // take data from subscribers
  if (accel_brake_value_source_ == ACCEL_BRAKE_SOURCE::STATUS) {
    ActuationStatusStamped::ConstSharedPtr actuation_status_ptr = actuation_status_sub_.takeData();
    if (!actuation_status_ptr) return false;
    take_actuation_status(actuation_status_ptr);
  }
  // take actuation data
  if (accel_brake_value_source_ == ACCEL_BRAKE_SOURCE::COMMAND) {
    ActuationCommandStamped::ConstSharedPtr actuation_cmd_ptr = actuation_cmd_sub_.takeData();
    if (!actuation_cmd_ptr) return false;
    take_actuation_command(actuation_cmd_ptr);
  }

  // take velocity data
  VelocityReport::ConstSharedPtr velocity_ptr = velocity_sub_.takeData();
  if (!velocity_ptr) return false;
  take_velocity(velocity_ptr);

  // take steer data
  steer_ptr_ = steer_sub_.takeData();

  /* valid check */
  // data check
  if (
    !twist_ptr_ || !steer_ptr_ || !accel_pedal_ptr_ || !brake_pedal_ptr_ ||
    !delayed_accel_pedal_ptr_ || !delayed_brake_pedal_ptr_) {
    // lack of data
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 5000, "lack of topics (twist, steer, accel, brake)");
    lack_of_data_count_++;
    return false;
  }
  return true;
}

void AccelBrakeMapCalibrator::fetch_data()
{
  update_count_++;

  RCLCPP_DEBUG_STREAM_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "map updating... count: " << update_success_count_ << " / " << update_count_ << "\n\t"
                              << "lack_of_data_count: " << lack_of_data_count_ << "\n\t"
                              << " failed_to_get_pitch_count: " << failed_to_get_pitch_count_
                              << "\n\t"
                              << "too_large_pitch_count: " << too_large_pitch_count_ << "\n\t"
                              << " too_low_speed_count: " << too_low_speed_count_ << "\n\t"
                              << "too_large_steer_count: " << too_large_steer_count_ << "\n\t"
                              << "too_large_jerk_count: " << too_large_jerk_count_ << "\n\t"
                              << "invalid_acc_brake_count: " << invalid_acc_brake_count_ << "\n\t"
                              << "too_large_pedal_spd_count: " << too_large_pedal_spd_count_
                              << "\n\t"
                              << "update_fail_count_: " << update_fail_count_ << "\n");

  // if cannot get data, return this callback
  if (!take_data()) return;

  debug_values_.data.at(CURRENT_STEER) = steer_ptr_->steering_tire_angle;

  // data check 2
  if (
    is_timeout(twist_ptr_->header.stamp, timeout_sec_) ||
    is_timeout(steer_ptr_->stamp, timeout_sec_) || is_timeout(accel_pedal_ptr_, timeout_sec_) ||
    is_timeout(brake_pedal_ptr_, timeout_sec_)) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 5000, "timeout of topics (twist, steer, accel, brake)");
    lack_of_data_count_++;
    return;
  }

  // data check 3
  if (!get_current_pitch_from_tf(&pitch_)) {
    // cannot get pitch
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "cannot get pitch");
    failed_to_get_pitch_count_++;
    return;
  }

  /* write data to log */
  if (pedal_accel_graph_output_) {
    add_log_to_csv(
      &output_log_, rclcpp::Time(twist_ptr_->header.stamp).seconds(), twist_ptr_->twist.linear.x,
      acceleration_, get_pitch_compensated_acceleration(), delayed_accel_pedal_ptr_->data,
      delayed_brake_pedal_ptr_->data, accel_pedal_speed_, brake_pedal_speed_, pitch_,
      steer_ptr_->steering_tire_angle, jerk_, full_original_accel_rmse_, part_original_accel_rmse_,
      new_accel_rmse_);
  }

  /* publish map  & debug_values*/
  publish_map(accel_map_value_, brake_map_value_, "original");
  publish_map(update_accel_map_value_, update_brake_map_value_, "update");
  publish_offset_cov_map(accel_offset_covariance_value_, brake_offset_covariance_value_);
  publish_count_map();
  publish_index();
  publish_update_suggest_flag();
  debug_pub_->publish(debug_values_);
  publish_float32("current_map_error", part_original_accel_rmse_);
  publish_float32("updated_map_error", new_accel_rmse_);
  publish_float32(
    "map_error_ratio",
    part_original_accel_rmse_ != 0.0 ? new_accel_rmse_ / part_original_accel_rmse_ : 1.0);

  // -- processing start --

  /* initialize */
  debug_values_.data.at(SUCCESS_TO_UPDATE) = false;
  update_success_ = false;

  // twist check
  if (twist_ptr_->twist.linear.x < velocity_min_threshold_) {
    // too low speed ( or backward velocity)
    too_low_speed_count_++;
    return;
  }

  // accel / brake map evaluation (do not evaluate when the car stops)
  execute_evaluation();

  // pitch check
  if (std::fabs(pitch_) > max_pitch_threshold_) {
    // too large pitch
    too_large_pitch_count_++;
    return;
  }

  // steer check
  if (std::fabs(steer_ptr_->steering_tire_angle) > max_steer_threshold_) {
    // too large steer
    too_large_steer_count_++;
    return;
  }

  // jerk check
  if (std::fabs(jerk_) > max_jerk_threshold_) {
    // too large jerk
    too_large_jerk_count_++;
    return;
  }

  // pedal check
  if (
    delayed_accel_pedal_ptr_->data > std::numeric_limits<double>::epsilon() &&
    delayed_brake_pedal_ptr_->data > std::numeric_limits<double>::epsilon()) {
    // both (accel/brake) output
    RCLCPP_DEBUG_STREAM_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "invalid pedal value (Both of accel output and brake output area not zero. )");
    invalid_acc_brake_count_++;
    return;
  }

  // pedal speed check
  if (
    std::fabs(accel_pedal_speed_) > pedal_velocity_thresh_ ||
    std::fabs(brake_pedal_speed_) > pedal_velocity_thresh_) {
    // too large pedal speed
    too_large_pedal_spd_count_++;
    return;
  }

  /* update map */
  if (update_accel_brake_map()) {
    update_success_count_++;
    debug_values_.data.at(SUCCESS_TO_UPDATE) = true;
    update_success_ = true;
  } else {
    update_fail_count_++;
  }
}

void AccelBrakeMapCalibrator::timer_callback_output_csv()
{
  // write accel/ brake map to file
  const auto ros_time = std::to_string(this->now().seconds());
  write_map_to_csv(
    accel_vel_index_, accel_pedal_index_, update_accel_map_value_, output_accel_file_);
  write_map_to_csv(
    brake_vel_index_, brake_pedal_index_, update_brake_map_value_, output_brake_file_);
  if (progress_file_output_) {
    write_map_to_csv(
      accel_vel_index_, accel_pedal_index_, update_accel_map_value_,
      output_accel_file_ + "_" + ros_time);
    write_map_to_csv(
      brake_vel_index_, brake_pedal_index_, update_brake_map_value_,
      output_brake_file_ + "_" + ros_time);
    write_map_to_csv(
      accel_vel_index_, accel_pedal_index_, accel_map_value_, output_accel_file_ + "_original");
    write_map_to_csv(
      brake_vel_index_, brake_pedal_index_, brake_map_value_, output_brake_file_ + "_original");
  }

  // update newest accel / brake map
  // check file existence
  std::ifstream af(output_accel_file_);
  std::ifstream bf(output_brake_file_);
  if (!af.is_open() || !bf.is_open()) {
    RCLCPP_WARN(get_logger(), "Accel map or brake map does not exist");
    return;
  }

  new_accel_map_ = AccelMap();
  if (!new_accel_map_.readAccelMapFromCSV(output_accel_file_)) {
    RCLCPP_WARN(get_logger(), "Cannot read accelmap. csv path = %s. ", output_accel_file_.c_str());
  }
  new_brake_map_ = BrakeMap();
  if (!new_brake_map_.readBrakeMapFromCSV(output_brake_file_)) {
    RCLCPP_WARN(get_logger(), "Cannot read brakemap. csv path = %s. ", output_brake_file_.c_str());
  }
}

void AccelBrakeMapCalibrator::take_velocity(const VelocityReport::ConstSharedPtr msg)
{
  // convert velocity-report to twist-stamped
  auto twist_msg = std::make_shared<TwistStamped>();
  twist_msg->header = msg->header;
  twist_msg->twist.linear.x = msg->longitudinal_velocity;
  twist_msg->twist.linear.y = msg->lateral_velocity;
  twist_msg->twist.angular.z = msg->heading_rate;

  if (!twist_vec_.empty()) {
    const auto past_msg = get_nearest_time_data_from_vec(twist_msg, dif_twist_time_, twist_vec_);
    const double raw_acceleration = get_accel(past_msg, twist_msg);
    acceleration_ = lowpass(acceleration_, raw_acceleration, 0.25);
    acceleration_time_ = rclcpp::Time(msg->header.stamp).seconds();
    debug_values_.data.at(CURRENT_RAW_ACCEL) = static_cast<float>(raw_acceleration);
    debug_values_.data.at(CURRENT_ACCEL) = static_cast<float>(acceleration_);

    // calculate jerk
    if (
      this->now().seconds() - pre_acceleration_time_ > timeout_sec_ ||
      (acceleration_time_ - pre_acceleration_time_) <= std::numeric_limits<double>::epsilon()) {
      RCLCPP_DEBUG_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "cannot calculate jerk");
      // does not update jerk
    } else {
      const double raw_jerk = get_jerk();
      // to avoid to get delayed jerk, set high gain
      jerk_ = lowpass(jerk_, raw_jerk, 0.5);
    }
    debug_values_.data.at(CURRENT_JERK) = static_cast<float>(jerk_);
    pre_acceleration_ = acceleration_;
    pre_acceleration_time_ = acceleration_time_;
  }

  debug_values_.data.at(CURRENT_SPEED) = static_cast<float>(twist_msg->twist.linear.x);
  twist_ptr_ = twist_msg;
  push_data_to_vec(twist_msg, twist_vec_max_size_, &twist_vec_);
}

void AccelBrakeMapCalibrator::take_actuation(
  const std_msgs::msg::Header & header, const double accel, const double brake)
{
  // get accel data
  accel_pedal_ptr_ = std::make_shared<DataStamped>(accel, rclcpp::Time(header.stamp));
  if (!accel_pedal_vec_.empty()) {
    const auto past_accel_ptr =
      get_nearest_time_data_from_vec(accel_pedal_ptr_, dif_pedal_time_, accel_pedal_vec_);
    const double raw_accel_pedal_speed =
      get_pedal_speed(past_accel_ptr, accel_pedal_ptr_, accel_pedal_speed_);
    accel_pedal_speed_ = lowpass(accel_pedal_speed_, raw_accel_pedal_speed, 0.5);
    debug_values_.data.at(CURRENT_RAW_ACCEL_SPEED) = static_cast<float>(raw_accel_pedal_speed);
    debug_values_.data.at(CURRENT_ACCEL_SPEED) = static_cast<float>(accel_pedal_speed_);
  }
  debug_values_.data.at(CURRENT_ACCEL_PEDAL) = static_cast<float>(accel_pedal_ptr_->data);
  push_data_to_vec(accel_pedal_ptr_, pedal_vec_max_size_, &accel_pedal_vec_);
  delayed_accel_pedal_ptr_ =
    get_nearest_time_data_from_vec(accel_pedal_ptr_, pedal_to_accel_delay_, accel_pedal_vec_);

  // get brake data
  brake_pedal_ptr_ = std::make_shared<DataStamped>(brake, rclcpp::Time(header.stamp));
  if (!brake_pedal_vec_.empty()) {
    const auto past_brake_ptr =
      get_nearest_time_data_from_vec(brake_pedal_ptr_, dif_pedal_time_, brake_pedal_vec_);
    const double raw_brake_pedal_speed =
      get_pedal_speed(past_brake_ptr, brake_pedal_ptr_, brake_pedal_speed_);
    brake_pedal_speed_ = lowpass(brake_pedal_speed_, raw_brake_pedal_speed, 0.5);
    debug_values_.data.at(CURRENT_RAW_BRAKE_SPEED) = static_cast<float>(raw_brake_pedal_speed);
    debug_values_.data.at(CURRENT_BRAKE_SPEED) = static_cast<float>(brake_pedal_speed_);
  }
  debug_values_.data.at(CURRENT_BRAKE_PEDAL) = static_cast<float>(brake_pedal_ptr_->data);
  push_data_to_vec(brake_pedal_ptr_, pedal_vec_max_size_, &brake_pedal_vec_);
  delayed_brake_pedal_ptr_ =
    get_nearest_time_data_from_vec(brake_pedal_ptr_, pedal_to_accel_delay_, brake_pedal_vec_);
}

void AccelBrakeMapCalibrator::take_actuation_command(
  const ActuationCommandStamped::ConstSharedPtr msg)
{
  const auto header = msg->header;
  const auto accel = msg->actuation.accel_cmd;
  const auto brake = msg->actuation.brake_cmd;
  take_actuation(header, accel, brake);
}

void AccelBrakeMapCalibrator::take_actuation_status(
  const ActuationStatusStamped::ConstSharedPtr msg)
{
  const auto header = msg->header;
  const auto accel = msg->status.accel_status;
  const auto brake = msg->status.brake_status;
  take_actuation(header, accel, brake);
}

bool AccelBrakeMapCalibrator::callback_update_map_service(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
  UpdateAccelBrakeMap::Request::SharedPtr req, UpdateAccelBrakeMap::Response::SharedPtr res)
{
  // if req.path is input, use this as the directory to set updated maps
  std::string update_map_dir = req->path.empty() ? csv_default_map_dir_ : req->path;

  RCLCPP_INFO_STREAM(get_logger(), "update accel/brake map. directory: " << update_map_dir);
  const auto accel_map_file = update_map_dir + "/accel_map.csv";
  const auto brake_map_file = update_map_dir + "/brake_map.csv";
  if (
    write_map_to_csv(
      accel_vel_index_, accel_pedal_index_, update_accel_map_value_, accel_map_file) &&
    write_map_to_csv(
      brake_vel_index_, brake_pedal_index_, update_brake_map_value_, brake_map_file)) {
    res->success = true;
    res->message =
      "Data has been successfully saved on " + update_map_dir + "/(accel/brake)_map.csv";
  } else {
    res->success = false;
    res->message = "Failed to save data. Maybe invalid path?";
  }
  return true;
}

double AccelBrakeMapCalibrator::lowpass(
  const double original, const double current, const double gain)
{
  return current * gain + original * (1.0 - gain);
}

double AccelBrakeMapCalibrator::get_pedal_speed(
  const DataStampedPtr & prev_pedal, const DataStampedPtr & current_pedal,
  const double prev_pedal_speed)
{
  const double dt = (current_pedal->data_time - prev_pedal->data_time).seconds();
  if (dt < 1e-03) {
    // invalid pedal info. return prev pedal speed
    return prev_pedal_speed;
  }

  const double d_pedal = current_pedal->data - prev_pedal->data;
  return d_pedal / dt;
}

double AccelBrakeMapCalibrator::get_accel(
  const TwistStamped::ConstSharedPtr & prev_twist,
  const TwistStamped::ConstSharedPtr & current_twist) const
{
  const double dt =
    (rclcpp::Time(current_twist->header.stamp) - rclcpp::Time(prev_twist->header.stamp)).seconds();
  if (dt < 1e-03) {
    // invalid twist. return prev acceleration
    return acceleration_;
  }
  const double dv = current_twist->twist.linear.x - prev_twist->twist.linear.x;
  return std::min(std::max(min_accel_, dv / dt), max_accel_);
}

double AccelBrakeMapCalibrator::get_jerk()
{
  const double jerk =
    (acceleration_ - pre_acceleration_) / (acceleration_time_ - pre_acceleration_time_);
  return std::min(std::max(-max_jerk_, jerk), max_jerk_);
}

bool AccelBrakeMapCalibrator::index_value_search(
  const std::vector<double> & value_index, const double value, const double value_thresh,
  int * searched_index) const
{
  if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_FOUR_CELL_AROUND) {
    /*  return lower left index.
    +-------+   +:grid
    |    *  |   *:given data
    |       |   o:index to return
    o-------+
    */
    if (value < 0) {
      std::cerr << "error : invalid value." << std::endl;
      return false;
    }
    double old_diff_value = -999;  // TODO(Hirano)
    for (std::size_t i = 0; i < value_index.size(); i++) {
      const double diff_value = value_index.at(i) - value;
      if (diff_value <= 0 && old_diff_value < diff_value) {
        *searched_index = static_cast<int>(i);
        old_diff_value = diff_value;
      } else {
        return true;
      }
    }
  } else {
    for (std::size_t i = 0; i < value_index.size(); i++) {
      const double diff_value = std::fabs(value_index.at(i) - value);
      if (diff_value <= value_thresh) {
        *searched_index = static_cast<int>(i);
        return true;
      }
    }
  }
  return false;
}

int AccelBrakeMapCalibrator::nearest_value_search(
  const std::vector<double> & value_index, const double value)
{
  double max_dist = std::numeric_limits<double>::max();
  int nearest_idx = 0;

  for (std::size_t i = 0; i < value_index.size(); i++) {
    const double dist = std::fabs(value - value_index.at(i));
    if (max_dist > dist) {
      nearest_idx = static_cast<int>(i);
      max_dist = dist;
    }
  }
  return nearest_idx;
}

int AccelBrakeMapCalibrator::nearest_pedal_search()
{
  bool accel_mode = (delayed_accel_pedal_ptr_->data > std::numeric_limits<double>::epsilon());

  int nearest_idx = 0;
  if (accel_mode) {
    nearest_idx = nearest_value_search(accel_pedal_index_, delayed_accel_pedal_ptr_->data);
  } else {
    nearest_idx = nearest_value_search(brake_pedal_index_, delayed_brake_pedal_ptr_->data);
  }

  return get_unified_index_from_accel_brake_index(accel_mode, nearest_idx);
}

int AccelBrakeMapCalibrator::nearest_vel_search()
{
  const double current_vel = twist_ptr_->twist.linear.x;
  return nearest_value_search(accel_vel_index_, current_vel);
}

void AccelBrakeMapCalibrator::take_consistency_of_accel_map()
{
  const double bit = std::pow(1e-01, precision_);
  for (std::size_t ped_idx = 0; ped_idx < update_accel_map_value_.size() - 1; ped_idx++) {
    for (std::size_t vel_idx = update_accel_map_value_.at(0).size() - 1;; vel_idx--) {
      if (vel_idx == 0) break;

      const double current_acc = update_accel_map_value_.at(ped_idx).at(vel_idx);
      const double next_ped_acc = update_accel_map_value_.at(ped_idx + 1).at(vel_idx);
      const double prev_vel_acc = update_accel_map_value_.at(ped_idx).at(vel_idx - 1);

      if (current_acc + bit >= prev_vel_acc) {
        // the higher the velocity, the lower the acceleration
        update_accel_map_value_.at(ped_idx).at(vel_idx - 1) = current_acc + bit;
      }

      if (current_acc + bit >= next_ped_acc) {
        // the higher the accel pedal, the higher the acceleration
        update_accel_map_value_.at(ped_idx + 1).at(vel_idx) = current_acc + bit;
      }
    }
  }
}

void AccelBrakeMapCalibrator::take_consistency_of_brake_map()
{
  const double bit = std::pow(1e-01, precision_);
  for (std::size_t ped_idx = 0; ped_idx < update_brake_map_value_.size() - 1; ped_idx++) {
    for (std::size_t vel_idx = 0; vel_idx < update_brake_map_value_.at(0).size() - 1; vel_idx++) {
      const double current_acc = update_brake_map_value_.at(ped_idx).at(vel_idx);
      const double next_ped_acc = update_brake_map_value_.at(ped_idx + 1).at(vel_idx);
      const double next_vel_acc = update_brake_map_value_.at(ped_idx).at(vel_idx + 1);

      if (current_acc <= next_vel_acc) {
        // the higher the velocity, the lower the acceleration
        update_brake_map_value_.at(ped_idx).at(vel_idx + 1) = current_acc - bit;
      }

      if (current_acc <= next_ped_acc) {
        // the higher the brake pedal, the lower the acceleration
        update_brake_map_value_.at(ped_idx + 1).at(vel_idx) = current_acc - bit;
      }
    }
  }
}

bool AccelBrakeMapCalibrator::update_accel_brake_map()
{
  // get pedal index
  bool accel_mode = false;
  int accel_pedal_index = 0;
  int brake_pedal_index = 0;
  int accel_vel_index = 0;
  int brake_vel_index = 0;

  accel_mode = (delayed_accel_pedal_ptr_->data > std::numeric_limits<double>::epsilon());

  if (
    accel_mode && !index_value_search(
                    accel_pedal_index_, delayed_accel_pedal_ptr_->data, pedal_diff_threshold_,
                    &accel_pedal_index)) {
    // not match accel pedal output to pedal value in index
    return false;
  }

  if (
    !accel_mode && !index_value_search(
                     brake_pedal_index_, delayed_brake_pedal_ptr_->data, pedal_diff_threshold_,
                     &brake_pedal_index)) {
    // not match accel pedal output to pedal value in index
    return false;
  }

  if (
    accel_mode &&
    !index_value_search(
      accel_vel_index_, twist_ptr_->twist.linear.x, velocity_diff_threshold_, &accel_vel_index)) {
    // not match current velocity to velocity value in index
    return false;
  }

  if (
    !accel_mode &&
    !index_value_search(
      brake_vel_index_, twist_ptr_->twist.linear.x, velocity_diff_threshold_, &brake_vel_index)) {
    // not match current velocity to velocity value in index
    return false;
  }

  // update map
  execute_update(
    accel_mode, accel_pedal_index, accel_vel_index, brake_pedal_index, brake_vel_index);

  // when update 0 pedal index, update another map
  if (accel_mode && accel_pedal_index == 0) {
    // copy accel map value to brake map value
    update_brake_map_value_.at(accel_pedal_index).at(accel_vel_index) =
      update_accel_map_value_.at(accel_pedal_index).at(accel_vel_index);
    if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_FOUR_CELL_AROUND) {
      update_brake_map_value_.at(accel_pedal_index).at(accel_vel_index + 1) =
        update_accel_map_value_.at(accel_pedal_index).at(accel_vel_index + 1);
    }
  } else if (!accel_mode && brake_pedal_index == 0) {
    // copy brake map value to accel map value
    update_accel_map_value_.at(brake_pedal_index).at(brake_vel_index) =
      update_brake_map_value_.at(brake_pedal_index).at(brake_vel_index);
    if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_FOUR_CELL_AROUND) {
      update_accel_map_value_.at(brake_pedal_index).at(brake_vel_index + 1) =
        update_brake_map_value_.at(brake_pedal_index).at(brake_vel_index + 1);
    }
  }

  // take consistency of map
  take_consistency_of_accel_map();
  take_consistency_of_brake_map();

  return true;
}

void AccelBrakeMapCalibrator::execute_update(
  const bool accel_mode, const int accel_pedal_index, const int accel_vel_index,
  const int brake_pedal_index, const int brake_vel_index)
{
  const double measured_acc = acceleration_ - get_pitch_compensated_acceleration();
  const double map_acc = accel_mode
                           ? update_accel_map_value_.at(accel_pedal_index).at(accel_vel_index)
                           : update_brake_map_value_.at(brake_pedal_index).at(brake_vel_index);
  RCLCPP_DEBUG_STREAM(get_logger(), "measured_acc: " << measured_acc << ", map_acc: " << map_acc);

  if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_EACH_CELL) {
    update_each_val_offset(
      accel_mode, accel_pedal_index, accel_vel_index, brake_pedal_index, brake_vel_index,
      measured_acc, map_acc);
  } else if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_TOTAL) {
    update_total_map_offset(measured_acc, map_acc);
  } else if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_FOUR_CELL_AROUND) {
    update_four_cell_around_offset(
      accel_mode, accel_pedal_index, accel_vel_index, brake_pedal_index, brake_vel_index,
      measured_acc);
  }

  // add accel data to map
  accel_mode
    ? map_value_data_.at(get_unified_index_from_accel_brake_index(true, accel_pedal_index))
        .at(accel_vel_index)
        .emplace_back(measured_acc)
    : map_value_data_.at(get_unified_index_from_accel_brake_index(false, brake_pedal_index))
        .at(brake_vel_index)
        .emplace_back(measured_acc);
}

bool AccelBrakeMapCalibrator::update_four_cell_around_offset(
  const bool accel_mode, const int accel_pedal_index, const int accel_vel_index,
  const int brake_pedal_index, const int brake_vel_index, const double measured_acc)
{
  // pre-defined
  static Map accelMapOffsetVec_(
    accel_map_value_.size(), std::vector<double>(accel_map_value_.at(0).size(), map_offset_));
  static Map brakeMapOffsetVec_(
    brake_map_value_.size(), std::vector<double>(accel_map_value_.at(0).size(), map_offset_));
  static std::vector<std::vector<Eigen::MatrixXd>> accelCovarianceMat_(
    accel_map_value_.size() - 1,
    std::vector<Eigen::MatrixXd>(
      accel_map_value_.at(0).size() - 1, Eigen::MatrixXd::Identity(4, 4) * covariance_));
  static std::vector<std::vector<Eigen::MatrixXd>> brakeCovarianceMat_(
    brake_map_value_.size() - 1,
    std::vector<Eigen::MatrixXd>(
      accel_map_value_.at(0).size() - 1, Eigen::MatrixXd::Identity(4, 4) * covariance_));

  auto & update_map_value = accel_mode ? update_accel_map_value_ : update_brake_map_value_;
  auto & offset_covariance_value =
    accel_mode ? accel_offset_covariance_value_ : brake_offset_covariance_value_;
  const auto & map_value = accel_mode ? accel_map_value_ : brake_map_value_;
  const auto & pedal_index = accel_mode ? accel_pedal_index : brake_pedal_index;
  const auto & pedal_index_ = accel_mode ? accel_pedal_index_ : brake_pedal_index_;
  const auto & vel_index = accel_mode ? accel_vel_index : brake_vel_index;
  const auto & vel_index_ = accel_mode ? accel_vel_index_ : brake_vel_index_;
  const auto & delayed_pedal =
    accel_mode ? delayed_accel_pedal_ptr_->data : delayed_brake_pedal_ptr_->data;
  auto & map_offset_vec = accel_mode ? accelMapOffsetVec_ : brakeMapOffsetVec_;
  auto & covariance_mat = accel_mode ? accelCovarianceMat_ : brakeCovarianceMat_;
  auto & data_mean_mat = accel_mode ? accel_data_mean_mat_ : brake_data_mean_mat_;
  auto & data_covariance_mat = accel_mode ? accel_data_covariance_mat_ : brake_data_covariance_mat_;
  auto & data_weighted_num = accel_mode ? accel_data_num_ : brake_data_num_;

  const double zll = update_map_value.at(pedal_index + 0).at(vel_index + 0);
  const double zhl = update_map_value.at(pedal_index + 1).at(vel_index + 0);
  const double zlh = update_map_value.at(pedal_index + 0).at(vel_index + 1);
  const double zhh = update_map_value.at(pedal_index + 1).at(vel_index + 1);

  const double xl = pedal_index_.at(pedal_index + 0);
  const double xh = pedal_index_.at(pedal_index + 1);
  const double rx = (delayed_pedal - xl) / (xh - xl);
  const double yl = vel_index_.at(vel_index + 0);
  const double yh = vel_index_.at(vel_index + 1);
  const double ry = (twist_ptr_->twist.linear.x - yl) / (yh - yl);

  Eigen::Vector4d phi(4);
  phi << (1 - rx) * (1 - ry), rx * (1 - ry), (1 - rx) * ry, rx * ry;

  Eigen::Vector4d theta(4);
  theta << zll, zhl, zlh, zhh;

  Eigen::Vector4d weighted_sum(4);
  weighted_sum << data_weighted_num(pedal_index + 0, vel_index + 0),
    data_weighted_num(pedal_index + 1, vel_index + 0),
    data_weighted_num(pedal_index + 0, vel_index + 1),
    data_weighted_num(pedal_index + 1, vel_index + 1);

  Eigen::Vector4d sigma(4);
  sigma << data_covariance_mat(pedal_index + 0, vel_index + 0),
    data_covariance_mat(pedal_index + 1, vel_index + 0),
    data_covariance_mat(pedal_index + 0, vel_index + 1),
    data_covariance_mat(pedal_index + 1, vel_index + 1);

  Eigen::Vector4d mean(4);
  mean << data_mean_mat(pedal_index + 0, vel_index + 0),
    data_mean_mat(pedal_index + 1, vel_index + 0), data_mean_mat(pedal_index + 0, vel_index + 1),
    data_mean_mat(pedal_index + 1, vel_index + 1);

  const int vel_idx_l = vel_index + 0;
  const int vel_idx_h = vel_index + 1;
  const int ped_idx_l = pedal_index + 0;
  const int ped_idx_h = pedal_index + 1;

  Eigen::VectorXd map_offset(4);
  map_offset(0) = map_offset_vec.at(ped_idx_l).at(vel_idx_l);
  map_offset(1) = map_offset_vec.at(ped_idx_h).at(vel_idx_l);
  map_offset(2) = map_offset_vec.at(ped_idx_l).at(vel_idx_h);
  map_offset(3) = map_offset_vec.at(ped_idx_h).at(vel_idx_h);

  Eigen::VectorXd updated_map_offset(4);

  Eigen::MatrixXd covariance = covariance_mat.at(ped_idx_l).at(vel_idx_l);

  /* calculate adaptive map offset */
  Eigen::MatrixXd matrixG(4, 4);
  Eigen::RowVectorXd phi_t(4);
  phi_t = phi.transpose();
  double rk = phi_t * covariance * phi;

  matrixG = covariance * phi / (forgetting_factor_ + rk);
  double beta = rk > 0 ? (forgetting_factor_ - (1 - forgetting_factor_) / rk) : 1;
  covariance = covariance - covariance * phi * phi_t * covariance / (1 / beta + rk);  // anti-windup
  double eta_hat = phi_t * theta;

  const double error_map_offset = measured_acc - eta_hat;
  updated_map_offset = map_offset + matrixG * error_map_offset;

  for (int i = 0; i < 4; i++) {
    const double pre_mean = mean(i);
    mean(i) = (weighted_sum(i) * pre_mean + error_map_offset) / (weighted_sum(i) + 1);
    sigma(i) =
      (weighted_sum(i) * (sigma(i) + pre_mean * pre_mean) + error_map_offset * error_map_offset) /
        (weighted_sum(i) + 1) -
      mean(i) * mean(i);
    weighted_sum(i) = weighted_sum(i) + 1;
  }

  /* input calculated result and update map */
  map_offset_vec.at(ped_idx_l).at(vel_idx_l) = updated_map_offset(0);
  map_offset_vec.at(ped_idx_h).at(vel_idx_l) = updated_map_offset(1);
  map_offset_vec.at(ped_idx_l).at(vel_idx_h) = updated_map_offset(2);
  map_offset_vec.at(ped_idx_h).at(vel_idx_h) = updated_map_offset(3);

  data_covariance_mat(ped_idx_l, vel_idx_l) = sigma(0);
  data_covariance_mat(ped_idx_h, vel_idx_l) = sigma(1);
  data_covariance_mat(ped_idx_l, vel_idx_h) = sigma(2);
  data_covariance_mat(ped_idx_h, vel_idx_h) = sigma(3);

  data_weighted_num(ped_idx_l, vel_idx_l) = weighted_sum(0);
  data_weighted_num(ped_idx_h, vel_idx_l) = weighted_sum(1);
  data_weighted_num(ped_idx_l, vel_idx_h) = weighted_sum(2);
  data_weighted_num(ped_idx_h, vel_idx_h) = weighted_sum(3);

  data_mean_mat(ped_idx_l, vel_idx_l) = mean(0);
  data_mean_mat(ped_idx_h, vel_idx_l) = mean(1);
  data_mean_mat(ped_idx_l, vel_idx_h) = mean(2);
  data_mean_mat(ped_idx_h, vel_idx_h) = mean(3);

  covariance_mat.at(ped_idx_l).at(vel_idx_l) = covariance;

  update_map_value.at(pedal_index + 0).at(vel_index + 0) =
    map_value.at(pedal_index + 0).at(vel_index + 0) + map_offset(0);
  update_map_value.at(pedal_index + 1).at(vel_index + 0) =
    map_value.at(pedal_index + 1).at(vel_index + 0) + map_offset(1);
  update_map_value.at(pedal_index + 0).at(vel_index + 1) =
    map_value.at(pedal_index + 0).at(vel_index + 1) + map_offset(2);
  update_map_value.at(pedal_index + 1).at(vel_index + 1) =
    map_value.at(pedal_index + 1).at(vel_index + 1) + map_offset(3);

  offset_covariance_value.at(pedal_index + 0).at(vel_index + 0) = std::sqrt(sigma(0));
  offset_covariance_value.at(pedal_index + 1).at(vel_index + 1) = std::sqrt(sigma(1));
  offset_covariance_value.at(pedal_index + 0).at(vel_index + 0) = std::sqrt(sigma(2));
  offset_covariance_value.at(pedal_index + 1).at(vel_index + 1) = std::sqrt(sigma(3));

  return true;
}

bool AccelBrakeMapCalibrator::update_each_val_offset(
  const bool accel_mode, const int accel_pedal_index, const int accel_vel_index,
  const int brake_pedal_index, const int brake_vel_index, const double measured_acc,
  const double map_acc)
{
  // pre-defined
  static Map map_offset_vec(
    accel_map_value_.size() + brake_map_value_.size() - 1,
    std::vector<double>(accel_map_value_.at(0).size(), map_offset_));
  static Map covariance_vec(
    accel_map_value_.size() + brake_map_value_.size() - 1,
    std::vector<double>(accel_map_value_.at(0).size(), covariance_));

  const int vel_idx = accel_mode ? accel_vel_index : brake_vel_index;
  int ped_idx = accel_mode ? accel_pedal_index : brake_pedal_index;
  ped_idx = get_unified_index_from_accel_brake_index(accel_mode, ped_idx);
  double map_offset = map_offset_vec.at(ped_idx).at(vel_idx);
  double covariance = covariance_vec.at(ped_idx).at(vel_idx);

  /* calculate adaptive map offset */
  const double phi = 1.0;
  covariance = (covariance - (covariance * phi * phi * covariance) /
                               (forgetting_factor_ + phi * covariance * phi)) /
               forgetting_factor_;

  const double coef = (covariance * phi) / (forgetting_factor_ + phi * covariance * phi);

  const double error_map_offset = measured_acc - map_acc;
  map_offset = map_offset + coef * error_map_offset;

  RCLCPP_DEBUG_STREAM(
    get_logger(), "index: " << ped_idx << ", " << vel_idx
                            << ": map_offset_ = " << map_offset_vec.at(ped_idx).at(vel_idx)
                            << " -> " << map_offset << "\t"
                            << " covariance = " << covariance);

  /* input calculated result and update map */
  map_offset_vec.at(ped_idx).at(vel_idx) = map_offset;
  covariance_vec.at(ped_idx).at(vel_idx) = covariance;
  if (accel_mode) {
    update_accel_map_value_.at(accel_pedal_index).at(accel_vel_index) =
      accel_map_value_.at(accel_pedal_index).at(accel_vel_index) + map_offset;
  } else {
    update_brake_map_value_.at(brake_pedal_index).at(brake_vel_index) =
      brake_map_value_.at(brake_pedal_index).at(brake_vel_index) + map_offset;
  }
  return true;
}

void AccelBrakeMapCalibrator::update_total_map_offset(
  const double measured_acc, const double map_acc)
{
  /* calculate adaptive map offset */
  const double phi = 1.0;
  covariance_ = (covariance_ - (covariance_ * phi * phi * covariance_) /
                                 (forgetting_factor_ + phi * covariance_ * phi)) /
                forgetting_factor_;

  const double coef = (covariance_ * phi) / (forgetting_factor_ + phi * covariance_ * phi);
  const double error_map_offset = measured_acc - map_acc;
  map_offset_ = map_offset_ + coef * error_map_offset;

  RCLCPP_DEBUG_STREAM(
    get_logger(), "map_offset_ = " << map_offset_ << "\t"
                                   << "covariance = " << covariance_);

  /* update map */
  for (std::size_t ped_idx = 0; ped_idx < update_accel_map_value_.size() - 1; ped_idx++) {
    for (std::size_t vel_idx = 0; vel_idx < update_accel_map_value_.at(0).size() - 1; vel_idx++) {
      update_accel_map_value_.at(ped_idx).at(vel_idx) =
        accel_map_value_.at(ped_idx).at(vel_idx) + map_offset_;
    }
  }
  for (std::size_t ped_idx = 0; ped_idx < update_brake_map_value_.size() - 1; ped_idx++) {
    for (std::size_t vel_idx = 0; vel_idx < update_brake_map_value_.at(0).size() - 1; vel_idx++) {
      update_brake_map_value_.at(ped_idx).at(vel_idx) =
        brake_map_value_.at(ped_idx).at(vel_idx) + map_offset_;
    }
  }
}

std::vector<double> AccelBrakeMapCalibrator::get_map_column_from_unified_index(
  const Map & accel_map_value, const Map & brake_map_value, const std::size_t index)
{
  if (index < brake_map_value.size()) {
    // input brake map value
    return brake_map_value.at(brake_map_value.size() - index - 1);
  }
  // input accel map value
  return accel_map_value.at(index - brake_map_value.size() + 1);
}

double AccelBrakeMapCalibrator::get_pedal_value_from_unified_index(const std::size_t index)
{
  if (index < brake_pedal_index_.size()) {
    // brake index ( minus )
    return -brake_pedal_index_.at(brake_pedal_index_.size() - index - 1);
  } else {
    // input accel map value
    return accel_pedal_index_.at(index - brake_pedal_index_.size() + 1);
  }
}

int AccelBrakeMapCalibrator::get_unified_index_from_accel_brake_index(
  const bool accel_map, const std::size_t index)
{
  // accel_map=true: accel_map; accel_map=false: brake_map
  if (accel_map) {
    // accel map
    return static_cast<int>(index) + static_cast<int>(brake_map_value_.size()) - 1;
  }
  // brake map
  return static_cast<int>(brake_map_value_.size()) - static_cast<int>(index) - 1;
}

double AccelBrakeMapCalibrator::get_pitch_compensated_acceleration() const
{
  // It works correctly only when the velocity is positive.
  constexpr double gravity = 9.80665;
  return gravity * std::sin(pitch_);
}

void AccelBrakeMapCalibrator::execute_evaluation()
{
  const double full_orig_accel_sq_error = calculate_accel_squared_error(
    delayed_accel_pedal_ptr_->data, delayed_brake_pedal_ptr_->data, twist_ptr_->twist.linear.x,
    accel_map_, brake_map_);
  push_data_to_vec(full_orig_accel_sq_error, full_mse_que_size_, &full_original_accel_mse_que_);
  full_original_accel_rmse_ = get_average(full_original_accel_mse_que_);
  // std::cerr << "rmse : " << sqrt(full_original_accel_rmse_) << std::endl;

  const double full_orig_accel_l1_error = calculate_accel_error_l1_norm(
    delayed_accel_pedal_ptr_->data, delayed_brake_pedal_ptr_->data, twist_ptr_->twist.linear.x,
    accel_map_, brake_map_);
  const double full_orig_accel_sq_l1_error = full_orig_accel_l1_error * full_orig_accel_l1_error;
  push_data_to_vec(full_orig_accel_l1_error, full_mse_que_size_, &full_original_accel_l1_que_);
  push_data_to_vec(
    full_orig_accel_sq_l1_error, full_mse_que_size_, &full_original_accel_sq_l1_que_);
  full_original_accel_error_l1norm_ = get_average(full_original_accel_l1_que_);

  /*calculate l1norm_covariance*/
  // const double full_original_accel_error_sql1_ = get_average(full_original_accel_sq_l1_que_);
  // std::cerr << "error_l1norm : " << full_original_accel_error_l1norm_ << std::endl;
  // std::cerr << "error_l1_cov : " <<
  // full_original_accel_error_sql1_-full_original_accel_error_l1norm_*full_original_accel_error_l1norm_
  // << std::endl;

  const double part_orig_accel_sq_error = calculate_accel_squared_error(
    delayed_accel_pedal_ptr_->data, delayed_brake_pedal_ptr_->data, twist_ptr_->twist.linear.x,
    accel_map_, brake_map_);
  push_data_to_vec(part_orig_accel_sq_error, part_mse_que_size_, &part_original_accel_mse_que_);
  part_original_accel_rmse_ = get_average(part_original_accel_mse_que_);

  const double new_accel_sq_error = calculate_accel_squared_error(
    delayed_accel_pedal_ptr_->data, delayed_brake_pedal_ptr_->data, twist_ptr_->twist.linear.x,
    new_accel_map_, new_brake_map_);
  push_data_to_vec(new_accel_sq_error, part_mse_que_size_, &new_accel_mse_que_);
  new_accel_rmse_ = get_average(new_accel_mse_que_);
}

double AccelBrakeMapCalibrator::calculate_estimated_acc(
  const double throttle, const double brake, const double vel, AccelMap & accel_map,
  BrakeMap & brake_map)
{
  const double pedal = throttle - brake;

  double estimated_acc = 0.0;
  if (pedal > 0.0) {
    accel_map.getAcceleration(pedal, vel, estimated_acc);
  } else {
    brake_map.getAcceleration(-pedal, vel, estimated_acc);
  }

  return estimated_acc;
}

double AccelBrakeMapCalibrator::calculate_accel_squared_error(
  const double throttle, const double brake, const double vel, AccelMap & accel_map,
  BrakeMap & brake_map)
{
  const double estimated_acc = calculate_estimated_acc(throttle, brake, vel, accel_map, brake_map);
  const double measured_acc = acceleration_ - get_pitch_compensated_acceleration();
  const double dif_acc = measured_acc - estimated_acc;
  return dif_acc * dif_acc;
}

double AccelBrakeMapCalibrator::calculate_accel_error_l1_norm(
  const double throttle, const double brake, const double vel, AccelMap & accel_map,
  BrakeMap & brake_map)
{
  const double estimated_acc = calculate_estimated_acc(throttle, brake, vel, accel_map, brake_map);
  const double measured_acc = acceleration_ - get_pitch_compensated_acceleration();
  const double dif_acc = measured_acc - estimated_acc;
  return abs(dif_acc);
}

void AccelBrakeMapCalibrator::push_data_to_que(
  const TwistStamped::ConstSharedPtr & data, const std::size_t max_size,
  std::queue<TwistStamped::ConstSharedPtr> * que)
{
  que->push(data);
  while (que->size() > max_size) {
    que->pop();
  }
}

template <class T>
void AccelBrakeMapCalibrator::push_data_to_vec(
  const T data, const std::size_t max_size, std::vector<T> * vec)
{
  vec->emplace_back(data);
  while (vec->size() > max_size) {
    vec->erase(vec->begin());
  }
}

template <class T>
T AccelBrakeMapCalibrator::get_nearest_time_data_from_vec(
  const T base_data, const double back_time, const std::vector<T> & vec)
{
  double nearest_time = std::numeric_limits<double>::max();
  const double target_time = rclcpp::Time(base_data->header.stamp).seconds() - back_time;
  T nearest_time_data;
  for (const auto & data : vec) {
    const double data_time = rclcpp::Time(data->header.stamp).seconds();
    const auto delta_time = std::abs(target_time - data_time);
    if (nearest_time > delta_time) {
      nearest_time_data = data;
      nearest_time = delta_time;
    }
  }
  return nearest_time_data;
}

DataStampedPtr AccelBrakeMapCalibrator::get_nearest_time_data_from_vec(
  DataStampedPtr base_data, const double back_time, const std::vector<DataStampedPtr> & vec)
{
  double nearest_time = std::numeric_limits<double>::max();
  const double target_time = base_data->data_time.seconds() - back_time;
  DataStampedPtr nearest_time_data;
  for (const auto & data : vec) {
    const double data_time = data->data_time.seconds();
    const auto delta_time = std::abs(target_time - data_time);
    if (nearest_time > delta_time) {
      nearest_time_data = data;
      nearest_time = delta_time;
    }
  }
  return nearest_time_data;
}

double AccelBrakeMapCalibrator::get_average(const std::vector<double> & vec)
{
  if (vec.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (const auto num : vec) {
    sum += num;
  }
  return sum / static_cast<double>(vec.size());
}

double AccelBrakeMapCalibrator::get_standard_deviation(const std::vector<double> & vec)
{
  if (vec.empty()) {
    return 0.0;
  }

  const double ave = get_average(vec);

  double sum = 0.0;
  for (const auto num : vec) {
    sum += std::pow(num - ave, 2);
  }
  return std::sqrt(sum / static_cast<double>(vec.size()));
}

bool AccelBrakeMapCalibrator::is_timeout(
  const builtin_interfaces::msg::Time & stamp, const double timeout_sec)
{
  const double dt = this->now().seconds() - rclcpp::Time(stamp).seconds();
  return dt > timeout_sec;
}

bool AccelBrakeMapCalibrator::is_timeout(
  const DataStampedPtr & data_stamped, const double timeout_sec)
{
  const double dt = (this->now() - data_stamped->data_time).seconds();
  return dt > timeout_sec;
}

OccupancyGrid AccelBrakeMapCalibrator::get_occ_msg(
  const std::string & frame_id, const double height, const double width, const double resolution,
  const std::vector<int8_t> & map_value)
{
  OccupancyGrid occ;
  occ.header.frame_id = frame_id;
  occ.header.stamp = this->now();
  occ.info.height = static_cast<double>(height);
  occ.info.width = static_cast<unsigned int>(width);
  occ.info.map_load_time = this->now();
  occ.info.origin.position.x = 0;
  occ.info.origin.position.y = 0;
  occ.info.origin.position.z = 0;
  occ.info.origin.orientation.x = 0;
  occ.info.origin.orientation.y = 0;
  occ.info.origin.orientation.z = 0;
  occ.info.origin.orientation.w = 1;
  occ.info.resolution = static_cast<float>(resolution);
  occ.data = map_value;
  return occ;
}

// function for diagnostics
void AccelBrakeMapCalibrator::check_update_suggest(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using CalibrationStatus = CalibrationStatus;
  CalibrationStatus accel_brake_map_status;
  accel_brake_map_status.target = CalibrationStatus::ACCEL_BRAKE_MAP;
  accel_brake_map_status.status = CalibrationStatus::NORMAL;
  int8_t level = DiagStatus::OK;
  std::string msg = "OK";

  if (!is_default_map_) {
    accel_brake_map_status.status = CalibrationStatus::UNAVAILABLE;
    level = DiagStatus::ERROR;
    msg = "Default map is not found in " + csv_default_map_dir_;
  }

  if (new_accel_mse_que_.size() < part_mse_que_size_ / 2) {
    // lack of data
    stat.summary(level, msg);
    calibration_status_pub_->publish(accel_brake_map_status);
    return;
  }

  const double rmse_rate = new_accel_rmse_ / part_original_accel_rmse_;
  if (rmse_rate < update_suggest_thresh_) {
    // The accuracy of original accel/brake map is low.
    // Suggest to update accel brake map
    level = DiagStatus::WARN;
    msg = "Accel/brake map Calibration is required.";
    accel_brake_map_status.status = CalibrationStatus::CALIBRATION_REQUIRED;
  }

  stat.summary(level, msg);
  calibration_status_pub_->publish(accel_brake_map_status);
}

// function for debug

void AccelBrakeMapCalibrator::publish_map(
  const Map & accel_map_value, const Map & brake_map_value, const std::string & publish_type)
{
  if (accel_map_value.at(0).size() != brake_map_value.at(0).size()) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Invalid map. The number of velocity index of accel map and brake map is different.");
    return;
  }
  const double h = static_cast<double>(accel_map_value.size()) + brake_map_value.size() -
                   1;  // pedal (accel_map_value(0) and brake_map_value(0) is same.)
  const double w = static_cast<double>(accel_map_value.at(0).size());  // velocity

  // publish occupancy map
  const int8_t max_occ_value = 100;
  std::vector<int8_t> int_map_value;
  int_map_value.resize(static_cast<std::vector<int>::size_type>(h * w));
  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      const double value =
        get_map_column_from_unified_index(accel_map_value_, brake_map_value_, i).at(j);
      // convert acc to 0~100 int value
      int8_t int_value =
        static_cast<int8_t>(max_occ_value * ((value - min_accel_) / (max_accel_ - min_accel_)));
      int_map_value.at(static_cast<std::vector<size_t>::size_type>(i * w + j)) =
        std::max(std::min(max_occ_value, int_value), (int8_t)0);
    }
  }

  if (publish_type == "original") {
    original_map_occ_pub_->publish(get_occ_msg("base_link", h, w, map_resolution_, int_map_value));
  } else {
    update_map_occ_pub_->publish(get_occ_msg("base_link", h, w, map_resolution_, int_map_value));
  }

  // publish raw map
  Float32MultiArray float_map;

  float_map.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  float_map.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  float_map.layout.dim[0].label = "height";
  float_map.layout.dim[1].label = "width";
  float_map.layout.dim[0].size = h;
  float_map.layout.dim[1].size = w;
  float_map.layout.dim[0].stride = h * w;
  float_map.layout.dim[1].stride = w;
  float_map.layout.data_offset = 0;
  std::vector<float>::size_type vec_size = static_cast<std::vector<float>::size_type>(h * w);
  std::vector<float> vec(vec_size, 0);
  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      vec[i * w + j] = static_cast<float>(
        get_map_column_from_unified_index(accel_map_value_, brake_map_value_, i).at(j));
    }
  }
  float_map.data = vec;
  if (publish_type == "original") {
    original_map_raw_pub_->publish(float_map);
  } else {
    update_map_raw_pub_->publish(float_map);
  }
}

void AccelBrakeMapCalibrator::publish_offset_cov_map(
  const Map accel_map_value, const Map brake_map_value)
{
  if (accel_map_value.at(0).size() != brake_map_value.at(0).size()) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Invalid map. The number of velocity index of accel map and brake map is different.");
    return;
  }
  const double h = accel_map_value.size() + brake_map_value.size() -
                   1;  // pedal (accel_map_value(0) and brake_map_value(0) is same.)
  const double w = accel_map_value.at(0).size();  // velocity

  // publish raw map
  Float32MultiArray float_map;

  float_map.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  float_map.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  float_map.layout.dim[0].label = "height";
  float_map.layout.dim[1].label = "width";
  float_map.layout.dim[0].size = h;
  float_map.layout.dim[1].size = w;
  float_map.layout.dim[0].stride = h * w;
  float_map.layout.dim[1].stride = w;
  float_map.layout.data_offset = 0;
  std::vector<float> vec(h * w, 0);
  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      std::vector<float>::size_type index = static_cast<std::vector<float>::size_type>(i * w + j);
      vec[index] = static_cast<float>(
        get_map_column_from_unified_index(accel_map_value, brake_map_value, i).at(j));
    }
  }
  float_map.data = vec;
  offset_covariance_pub_->publish(float_map);
}

void AccelBrakeMapCalibrator::publish_float32(const std::string & publish_type, const double val)
{
  Float32Stamped msg;
  msg.stamp = this->now();
  msg.data = static_cast<float>(val);
  if (publish_type == "current_map_error") {
    current_map_error_pub_->publish(msg);
  } else if (publish_type == "updated_map_error") {
    updated_map_error_pub_->publish(msg);
  } else {
    map_error_ratio_pub_->publish(msg);
  }
}

void AccelBrakeMapCalibrator::publish_count_map()
{
  if (accel_map_value_.at(0).size() != brake_map_value_.at(0).size()) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Invalid map. The number of velocity index of accel map and brake map is different.");
    return;
  }
  const double h = static_cast<double>(
    accel_map_value_.size() + brake_map_value_.size() -
    1);  // pedal (accel_map_value(0) and brake_map_value(0) is same.)

  const double w = static_cast<double>(accel_map_value_.at(0).size());  // velocity
  const int8_t max_occ_value = 100;

  std::vector<int8_t> count_map;
  std::vector<int8_t> ave_map;
  std::vector<int8_t> std_map;
  count_map.resize(static_cast<std::vector<int8_t>::size_type>(h * w));
  ave_map.resize(static_cast<std::vector<int8_t>::size_type>(h * w));
  std_map.resize(static_cast<std::vector<int8_t>::size_type>(h * w));

  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      const auto data_vec = map_value_data_.at(i).at(j);
      if (data_vec.empty()) {
        // input *UNKNOWN* value
        count_map.at(static_cast<std::vector<int8_t>::size_type>(i * w + j)) = -1;
        ave_map.at(static_cast<std::vector<int8_t>::size_type>(i * w + j)) = -1;
      } else {
        const auto count_rate =
          max_occ_value * (static_cast<double>(data_vec.size()) / max_data_count_);
        count_map.at(static_cast<std::vector<int8_t>::size_type>(i * w + j)) = static_cast<int8_t>(
          std::max(std::min(static_cast<int>(max_occ_value), static_cast<int>(count_rate)), 0));
        // calculate average
        {
          const double average = get_average(data_vec);
          int8_t int_average = static_cast<int8_t>(
            max_occ_value * ((average - min_accel_) / (max_accel_ - min_accel_)));
          ave_map.at(static_cast<std::vector<int8_t>::size_type>(i * w + j)) =
            std::max(std::min(max_occ_value, int_average), static_cast<int8_t>(0));
        }
        // calculate standard deviation
        {
          const double std_dev = get_standard_deviation(data_vec);
          const double max_std_dev = 0.2;
          const double min_std_dev = 0.0;
          int8_t int_std_dev = static_cast<uint8_t>(
            max_occ_value * ((std_dev - min_std_dev) / (max_std_dev - min_std_dev)));
          std_map.at(i * w + j) = std::max(std::min(max_occ_value, int_std_dev), (int8_t)0);
        }
      }
    }
  }

  // publish average map / standard dev map / count map
  data_ave_pub_->publish(get_occ_msg("base_link", h, w, map_resolution_, ave_map));
  data_std_pub_->publish(get_occ_msg("base_link", h, w, map_resolution_, std_map));
  data_count_pub_->publish(get_occ_msg("base_link", h, w, map_resolution_, count_map));

  // publish count with self pos map
  int nearest_pedal_idx = nearest_pedal_search();
  int nearest_vel_idx = nearest_vel_search();

  // input self pose (to max value / min value ) and publish this
  update_success_
    ? count_map.at(static_cast<std::vector<int8_t>::size_type>(
        nearest_pedal_idx * w + nearest_vel_idx)) = std::numeric_limits<int8_t>::max()
    : count_map.at(static_cast<std::vector<int8_t>::size_type>(
        nearest_pedal_idx * w + nearest_vel_idx)) = std::numeric_limits<int8_t>::min();
  data_count_with_self_pose_pub_->publish(
    get_occ_msg("base_link", h, w, map_resolution_, count_map));
}

void AccelBrakeMapCalibrator::publish_index()
{
  MarkerArray markers;
  const double h = static_cast<double>(accel_map_value_.size()) + brake_map_value_.size() -
                   1;  // pedal (accel_map_value(0) and brake_map_value(0) is same.)
  const double w = static_cast<double>(accel_map_value_.at(0).size());  // velocity

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = this->now();
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  std_msgs::msg::ColorRGBA color;
  color.a = 0.999;
  color.b = 0.1;
  color.g = 0.1;
  color.r = 0.1;
  marker.color = color;
  marker.frame_locked = true;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // pedal value
  for (int pedal_idx = 0; pedal_idx < h; pedal_idx++) {
    const double pedal_value = get_pedal_value_from_unified_index(pedal_idx);
    marker.ns = "occ_pedal_index";
    marker.id = pedal_idx;
    marker.pose.position.x = -map_resolution_ * 0.5;
    marker.pose.position.y = map_resolution_ * (0.5 + pedal_idx);
    marker.scale.z = 0.03;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << pedal_value;
    marker.text = stream.str();
    markers.markers.push_back(marker);
  }

  // pedal index name
  marker.ns = "occ_pedal_index";
  marker.id = static_cast<int>(h);
  marker.pose.position.x = -map_resolution_ * 0.5;
  marker.pose.position.y = map_resolution_ * (0.5 + h);
  marker.scale.z = 0.03;
  marker.text = "(accel_pedal/brake_pedal)";
  markers.markers.push_back(marker);

  // velocity value
  for (int vel_idx = 0; vel_idx < w; vel_idx++) {
    const double vel_value = accel_vel_index_.at(vel_idx);
    marker.ns = "occ_vel_index";
    marker.id = vel_idx;
    marker.pose.position.x = map_resolution_ * (0.5 + vel_idx);
    marker.pose.position.y = -map_resolution_ * 0.2;
    marker.scale.z = 0.02;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << vel_value;
    marker.text = stream.str() + "m/s";
    markers.markers.push_back(marker);
  }

  // velocity index name
  marker.ns = "occ_vel_index";
  marker.id = static_cast<int>(w);
  marker.pose.position.x = map_resolution_ * (0.5 + w);
  marker.pose.position.y = -map_resolution_ * 0.2;
  marker.scale.z = 0.02;
  marker.text = "(velocity)";
  markers.markers.push_back(marker);

  index_pub_->publish(markers);
}

void AccelBrakeMapCalibrator::publish_update_suggest_flag()
{
  std_msgs::msg::Bool update_suggest;

  if (new_accel_mse_que_.size() < part_mse_que_size_ / 2) {
    // lack of data
    update_suggest.data = false;
  } else {
    const double rmse_rate = new_accel_rmse_ / part_original_accel_rmse_;
    update_suggest.data = (rmse_rate < update_suggest_thresh_);
    if (update_suggest.data) {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "suggest to update accel/brake map. evaluation score = " << rmse_rate);
    }
  }

  update_suggest_pub_->publish(update_suggest);
}

bool AccelBrakeMapCalibrator::write_map_to_csv(
  std::vector<double> vel_index, std::vector<double> pedal_index, Map value_map,
  std::string filename)
{
  if (update_success_count_ == 0) {
    return false;
  }

  std::ofstream csv_file(filename);

  if (!csv_file.is_open()) {
    RCLCPP_WARN(get_logger(), "Failed to open csv file : %s", filename.c_str());
    return false;
  }

  csv_file << "default,";
  for (std::size_t v = 0; v < vel_index.size(); v++) {
    csv_file << vel_index.at(v);
    if (v != vel_index.size() - 1) {
      csv_file << ",";
    }
  }
  csv_file << "\n";

  for (std::size_t p = 0; p < pedal_index.size(); p++) {
    csv_file << pedal_index.at(p) << ",";
    for (std::size_t v = 0; v < vel_index.size(); v++) {
      csv_file << std::fixed << std::setprecision(precision_) << value_map.at(p).at(v);
      if (v != vel_index.size() - 1) {
        csv_file << ",";
      }
    }
    csv_file << "\n";
  }
  csv_file.close();
  RCLCPP_DEBUG_STREAM(get_logger(), "output map to " << filename);
  return true;
}

void AccelBrakeMapCalibrator::add_index_to_csv(std::ofstream * csv_file)
{
  *csv_file << "timestamp,velocity,accel,pitch_comp_accel,final_accel,accel_pedal,brake_pedal,"
            << "accel_pedal_speed,brake_pedal_speed,pitch,steer,jerk,full_original_accel_rmse, "
               "part_original_accel_rmse,new_accel_rmse, rmse_rate"
            << std::endl;
}

void AccelBrakeMapCalibrator::add_log_to_csv(
  std::ofstream * csv_file, const double & timestamp, const double velocity, const double accel,
  const double pitched_accel, const double accel_pedal, const double brake_pedal,
  const double accel_pedal_speed, const double brake_pedal_speed, const double pitch,
  const double steer, const double jerk, const double full_original_accel_mse,
  const double part_original_accel_mse, const double new_accel_mse)
{
  const double rmse_rate =
    part_original_accel_mse == 0.0 ? 1.0 : (new_accel_mse / part_original_accel_mse);
  *csv_file << timestamp << ", " << velocity << ", " << accel << ", " << pitched_accel << ", "
            << accel - pitched_accel << ", " << accel_pedal << ", " << brake_pedal << ", "
            << accel_pedal_speed << ", " << brake_pedal_speed << ", " << pitch << ", " << steer
            << ", " << jerk << ", " << full_original_accel_mse << ", " << part_original_accel_mse
            << ", " << new_accel_mse << "," << rmse_rate << std::endl;
}

}  // namespace autoware::accel_brake_map_calibrator
