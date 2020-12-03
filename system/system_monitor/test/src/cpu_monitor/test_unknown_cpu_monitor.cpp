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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <system_monitor/cpu_monitor/unknown_cpu_monitor.hpp>

using DiagStatus = diagnostic_msgs::DiagnosticStatus;

class TestCPUMonitor : public CPUMonitor
{
  friend class CPUMonitorTestSuite;

public:
  TestCPUMonitor(const ros::NodeHandle & nh, const ros::NodeHandle & pnh) : CPUMonitor(nh, pnh) {}

  void diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr & diag_msg)
  {
    array_ = *diag_msg;
  }

  void update(void) { updater_.force_update(); }

private:
  diagnostic_msgs::DiagnosticArray array_;
};

class CPUMonitorTestSuite : public ::testing::Test
{
public:
  CPUMonitorTestSuite() : nh_(""), pnh_("~") {}

protected:
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<TestCPUMonitor> monitor_;
  ros::Subscriber sub_;

  void SetUp(void)
  {
    monitor_ = std::make_unique<TestCPUMonitor>(nh_, pnh_);
    sub_ = nh_.subscribe("/diagnostics", 1000, &TestCPUMonitor::diagCallback, monitor_.get());
    monitor_->getTempNames();
    monitor_->getFreqNames();
  }

  void TearDown(void) {}
};

TEST_F(CPUMonitorTestSuite, test) { ASSERT_TRUE(true); }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "CPUMonitorTestNode");

  return RUN_ALL_TESTS();
}
