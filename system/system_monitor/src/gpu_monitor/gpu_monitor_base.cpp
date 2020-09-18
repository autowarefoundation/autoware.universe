/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
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

/**
 * @file nvml_gpu_monitor.cpp
 * @brief GPU monitor class
 */

#include <system_monitor/gpu_monitor/gpu_monitor_base.h>

GPUMonitorBase::GPUMonitorBase(const ros::NodeHandle & nh, const ros::NodeHandle & pnh)
: nh_(nh),
  pnh_(pnh),
  updater_(),
  hostname_(),
  temp_warn_(90.0),
  temp_error_(95.0),
  gpu_usage_warn_(0.90),
  gpu_usage_error_(1.00),
  memory_usage_warn_(0.95),
  memory_usage_error_(0.99)
{
  gethostname(hostname_, sizeof(hostname_));

  pnh_.param<float>("temp_warn", temp_warn_, 90.0);
  pnh_.param<float>("temp_error", temp_error_, 95.0);
  pnh_.param<float>("gpu_usage_warn", gpu_usage_warn_, 0.90);
  pnh_.param<float>("gpu_usage_error", gpu_usage_error_, 1.00);
  pnh_.param<float>("memory_usage_warn", memory_usage_warn_, 0.95);
  pnh_.param<float>("memory_usage_error", memory_usage_error_, 0.99);

  updater_.setHardwareID(hostname_);
  updater_.add("GPU Temperature", this, &GPUMonitorBase::checkTemp);
  updater_.add("GPU Usage", this, &GPUMonitorBase::checkUsage);
  updater_.add("GPU Memory Usage", this, &GPUMonitorBase::checkMemoryUsage);
  updater_.add("GPU Thermal Throttling", this, &GPUMonitorBase::checkThrottling);
  updater_.add("GPU Frequency", this, &GPUMonitorBase::checkFrequency);
}

void GPUMonitorBase::run(void)
{
  ros::Rate rate(1.0);

  while (ros::ok()) {
    ros::spinOnce();
    updater_.force_update();
    rate.sleep();
  }
}

void GPUMonitorBase::checkTemp(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  ROS_INFO("GPUMonitorBase::checkTemp not implemented.");
}

void GPUMonitorBase::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  ROS_INFO("GPUMonitorBase::checkUsage not implemented.");
}

void GPUMonitorBase::checkMemoryUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  ROS_INFO("GPUMonitorBase::checkMemoryUsage not implemented.");
}

void GPUMonitorBase::checkThrottling(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  ROS_INFO("GPUMonitorBase::checkThrottling not implemented.");
}

void GPUMonitorBase::checkFrequency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  ROS_INFO("GPUMonitorBase::checkFrequency not implemented.");
}
