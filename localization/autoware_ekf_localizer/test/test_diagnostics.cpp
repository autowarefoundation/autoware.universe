// Copyright 2023 Autoware Foundation
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

#include "autoware/ekf_localizer/diagnostics.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace autoware::ekf_localizer
{

TEST(TestEkfDiagnostics, check_process_activated)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  bool is_activated = true;
  stat = check_process_activated(is_activated);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  is_activated = false;
  stat = check_process_activated(is_activated);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(TestEkfDiagnostics, check_set_initialpose)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  bool is_set_initialpose = true;
  stat = check_set_initialpose(is_set_initialpose);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  is_set_initialpose = false;
  stat = check_set_initialpose(is_set_initialpose);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(TestEkfDiagnostics, check_measurement_updated)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const std::string measurement_type = "pose";  // not effect for stat.level
  const size_t no_update_count_threshold_warn = 50;
  const size_t no_update_count_threshold_error = 250;

  size_t no_update_count = 0;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  no_update_count = 1;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  no_update_count = 49;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  no_update_count = 50;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  no_update_count = 249;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  no_update_count = 250;
  stat = check_measurement_updated(
    measurement_type, no_update_count, no_update_count_threshold_warn,
    no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(TestEkfDiagnostics, check_measurement_queue_size)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const std::string measurement_type = "pose";  // not effect for stat.level

  size_t queue_size = 0;  // not effect for stat.level
  stat = check_measurement_queue_size(measurement_type, queue_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  queue_size = 1;  // not effect for stat.level
  stat = check_measurement_queue_size(measurement_type, queue_size);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST(TestEkfDiagnostics, check_measurement_delay_gate)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const std::string measurement_type = "pose";  // not effect for stat.level
  const double delay_time = 0.1;                // not effect for stat.level
  const double delay_time_threshold = 1.0;      // not effect for stat.level

  bool is_passed_delay_gate = true;
  stat = check_measurement_delay_gate(
    measurement_type, is_passed_delay_gate, delay_time, delay_time_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  is_passed_delay_gate = false;
  stat = check_measurement_delay_gate(
    measurement_type, is_passed_delay_gate, delay_time, delay_time_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(TestEkfDiagnostics, check_measurement_mahalanobis_gate)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  const std::string measurement_type = "pose";        // not effect for stat.level
  const double mahalanobis_distance = 0.1;            // not effect for stat.level
  const double mahalanobis_distance_threshold = 1.0;  // not effect for stat.level

  bool is_passed_mahalanobis_gate = true;
  stat = check_measurement_mahalanobis_gate(
    measurement_type, is_passed_mahalanobis_gate, mahalanobis_distance,
    mahalanobis_distance_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  is_passed_mahalanobis_gate = false;
  stat = check_measurement_mahalanobis_gate(
    measurement_type, is_passed_mahalanobis_gate, mahalanobis_distance,
    mahalanobis_distance_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(TestLocalizationErrorMonitorDiagnostics, merge_diagnostic_status)
{
  diagnostic_msgs::msg::DiagnosticStatus merged_stat;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> stat_array(2);

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(0).message = "OK";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(1).message = "OK";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(merged_stat.message, "OK");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(0).message = "WARN0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(1).message = "OK";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(merged_stat.message, "WARN0");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(0).message = "OK";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(1).message = "WARN1";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(merged_stat.message, "WARN1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(0).message = "WARN0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(1).message = "WARN1";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(merged_stat.message, "WARN0; WARN1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat_array.at(0).message = "OK";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(1).message = "ERROR1";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(merged_stat.message, "ERROR1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  stat_array.at(0).message = "WARN0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(1).message = "ERROR1";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(merged_stat.message, "WARN0; ERROR1");

  stat_array.at(0).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(0).message = "ERROR0";
  stat_array.at(1).level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  stat_array.at(1).message = "ERROR1";
  merged_stat = merge_diagnostic_status(stat_array);
  EXPECT_EQ(merged_stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(merged_stat.message, "ERROR0; ERROR1");
}

}  // namespace autoware::ekf_localizer
