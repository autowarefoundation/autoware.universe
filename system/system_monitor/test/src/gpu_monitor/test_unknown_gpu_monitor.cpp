// Copyright 2020 Autoware Foundation
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

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "system_monitor/gpu_monitor/unknown_gpu_monitor.hpp"

using DiagStatus = diagnostic_msgs::DiagnosticStatus;

class TestGPUMonitor : public GPUMonitor
{
  friend class GPUMonitorTestSuite;

public:
  TestGPUMonitor(const ros::NodeHandle & nh, const ros::NodeHandle & pnh) : GPUMonitor(nh, pnh) {}

  void diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr & diag_msg)
  {
    array_ = *diag_msg;
  }

  void update(void) { updater_.force_update(); }

private:
  diagnostic_msgs::DiagnosticArray array_;
};

class GPUMonitorTestSuite : public ::testing::Test
{
public:
  GPUMonitorTestSuite() : nh_(""), pnh_("~") {}

protected:
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<TestGPUMonitor> monitor_;
  ros::Subscriber sub_;

  void SetUp(void)
  {
    monitor_ = std::make_unique<TestGPUMonitor>(nh_, pnh_);
    sub_ = nh_.subscribe("/diagnostics", 1000, &TestGPUMonitor::diagCallback, monitor_.get());
  }

  void TearDown(void) {}
};

TEST_F(GPUMonitorTestSuite, test) { ASSERT_TRUE(true); }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "GPUMonitorTestNode");

  return RUN_ALL_TESTS();
}
