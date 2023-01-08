// Copyright 2020 Tier IV, Inc.
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

/**
 * @file hdd_monitor.h
 * @brief HDD monitor class
 */

#ifndef SYSTEM_MONITOR__HDD_MONITOR__HDD_MONITOR_HPP_
#define SYSTEM_MONITOR__HDD_MONITOR__HDD_MONITOR_HPP_

#include "hdd_reader/hdd_reader_common.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <boost/asio.hpp>
#include <boost/process.hpp>

#include <climits>
#include <map>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Parameters
 */
struct HddParam
{
  std::string part_device;                    //!< @brief Partition device
  std::string disk_device;                    //!< @brief Disk device
  float temp_warn{55.0};                      //!< @brief HDD temperature(DegC) to generate warning
  float temp_error{70.0};                     //!< @brief HDD temperature(DegC) to generate error
  uint64_t power_on_hours_warn{3000000};      //!< @brief HDD power on hours to generate warning
  uint64_t total_data_written_warn{4915200};  //!< @brief HDD total data written to generate warning
  float total_data_written_safety_factor{0.05};  //!< @brief Safety factor of HDD total data written
  uint32_t recovered_error_warn{1};   //!< @brief HDD recovered error count to generate warning
  uint32_t free_warn{5120};           //!< @brief HDD free space(MB) to generate warning
  uint32_t free_error{100};           //!< @brief HDD free space(MB) to generate error
  float read_data_rate_warn{360.0};   //!< @brief HDD data rate(MB/s) of read to generate warning
  float write_data_rate_warn{103.5};  //!< @brief HDD data rate(MB/s) of write to generate warning
  float read_iops_warn{63360.0};      //!< @brief HDD IOPS of read to generate warning
  float write_iops_warn{24120.0};     //!< @brief HDD IOPS of write to generate warning
  uint8_t temp_attribute_id{0xC2};    //!< @brief S.M.A.R.T attribute ID of temperature
  uint8_t power_on_hours_attribute_id{0x09};  //!< @brief S.M.A.R.T attribute ID of power on hours
  uint8_t total_data_written_attribute_id{
    0xF1};  //!< @brief S.M.A.R.T attribute ID of total data written
  uint8_t recovered_error_attribute_id{0xC3};  //!< @brief S.M.A.R.T attribute ID of recovered error
};

/**
 * @brief Statistics of block device
 */
struct DeviceStatistics
{
  uint64_t read_ios;       //!< @brief Number of read I/Os processed
  uint64_t read_sectors;   //!< @brief Number of sectors read
  uint64_t write_ios;      //!< @brief Number of write I/Os processed
  uint64_t write_sectors;  //!< @brief Number of sectors written
};

/**
 * @brief Statistics of HDD
 */
struct HddStatistics
{
  std::string device;                //!< @brief Disk device
  std::string error_message;         //!< @brief Error message
  float read_data_rate_MBs;          //!< @brief Data rate of read (MB/s)
  float write_data_rate_MBs;         //!< @brief Data rate of write (MB/s)
  float read_iops;                   //!< @brief IOPS of read
  float write_iops;                  //!< @brief IOPS of write
  DeviceStatistics last_statistics;  //!< @brief Last statistics of device
};

/**
 * @brief Type to check
 */
enum class CheckType {
  TEMPERATURE = 0,
  POWER_ON_HOURS,
  TOTAL_DATA_WRITTEN,
  RECOVERED_ERROR,
  READ_DATA_RATE,
  WRITE_DATA_RATE,
  READ_IOPS,
  WRITE_IOPS,
  SIZE
};

namespace bp = boost::process;
namespace local = boost::asio::local;

class HddMonitor : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param [in] options Options associated with this node.
   */
  explicit HddMonitor(const rclcpp::NodeOptions & options);

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief Check HDD connection
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_connection(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Check HDD temperature
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_smart_temperature(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Check HDD power on hours
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_smart_power_on_hours(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Check HDD total data written
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_smart_total_data_written(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Check HDD recovered error count
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_smart_recovered_error(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Check S.M.A.R.T. information
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   * @param [in] type Type to check
   */
  void check_smart(diagnostic_updater::DiagnosticStatusWrapper & stat, CheckType type);

  /**
   * @brief Set temperature data
   * @param [in] param Parameters
   * @param [in] info HDD information
   * @param [in] index Index number of HDD
   * @param [out] key Key value of diagnostic message
   * @param [out] value Value of diagnostic message
   */
  static int set_smart_temperature(
    const HddParam & param, const hdd_reader_service::HddInformation & info, int index,
    std::string & key, std::string & value);

  /**
   * @brief Set power on hours data
   * @param [in] param Parameters
   * @param [in] info HDD information
   * @param [in] index Index number of HDD
   * @param [out] key Key value of diagnostic message
   * @param [out] value Value of diagnostic message
   */
  static int set_smart_power_on_hours(
    const HddParam & param, const hdd_reader_service::HddInformation & info, int index,
    std::string & key, std::string & value);

  /**
   * @brief Set total data written data
   * @param [in] param Parameters
   * @param [in] info HDD information
   * @param [in] index Index number of HDD
   * @param [out] key Key value of diagnostic message
   * @param [out] value Value of diagnostic message
   */
  static int set_smart_total_data_written(
    const HddParam & param, const hdd_reader_service::HddInformation & info, int index,
    std::string & key, std::string & value);

  /**
   * @brief Set recovered error count data
   * @param [in] param Parameters
   * @param [in] info HDD information
   * @param [in] index Index number of HDD
   * @param [out] key Key value of diagnostic message
   * @param [out] value Value of diagnostic message
   */
  static int set_smart_recovered_error(
    const HddParam & param, const hdd_reader_service::HddInformation & info, int index,
    std::string & key, std::string & value);

  /**
   * @brief Check HDD usage
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_usage(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Run df(disk free) command
   * @param [in] partition Device name of partition
   * @param [out] is_out Output stream
   * @param [out] error_message Error message
   */
  int run_disk_free_command(
    const std::string & partition, bp::ipstream & is_out, std::string & error_message);

  /**
   * @brief Check HDD data rate of read
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_read_data_rate(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Check HDD data rate of write
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_write_data_rate(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Check HDD IOPS of read
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_read_iops(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Check HDD IOPS of write
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_write_iops(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Check HDD statistics
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   * @param [in] type Type to check
   */
  void check_statistics(diagnostic_updater::DiagnosticStatusWrapper & stat, CheckType type);

  /**
   * @brief Set rate of read data
   * @param [in] param Parameters
   * @param [in] stat Statistics of HDD
   * @param [in] index Index number of HDD
   * @param [out] key Key value of diagnostic message
   * @param [out] value Value of diagnostic message
   */
  static int set_statistics_read_data_rate(
    const HddParam & param, const HddStatistics & stat, int index, std::string & key,
    std::string & value);

  /**
   * @brief Set rate of write data
   * @param [in] param Parameters
   * @param [in] stat Statistics of HDD
   * @param [in] index Index number of HDD
   * @param [out] key Key value of diagnostic message
   * @param [out] value Value of diagnostic message
   */
  static int set_statistics_write_data_rate(
    const HddParam & param, const HddStatistics & stat, int index, std::string & key,
    std::string & value);

  /**
   * @brief Set IOPS of read data
   * @param [in] param Parameters
   * @param [in] stat Statistics of HDD
   * @param [in] index Index number of HDD
   * @param [out] key Key value of diagnostic message
   * @param [out] value Value of diagnostic message
   */
  static int set_statistics_read_iops(
    const HddParam & param, const HddStatistics & stat, int index, std::string & key,
    std::string & value);

  /**
   * @brief Set IOPS of write data
   * @param [in] param Parameters
   * @param [in] stat Statistics of HDD
   * @param [in] index Index number of HDD
   * @param [out] key Key value of diagnostic message
   * @param [out] value Value of diagnostic message
   */
  static int set_statistics_write_iops(
    const HddParam & param, const HddStatistics & stat, int index, std::string & key,
    std::string & value);

  /**
   * @brief Human readable size string to MB
   * @param [in] human Readable size string
   * @return Megabyte
   */
  int human_readable_to_mega_byte(const std::string & str);

  /**
   * @brief Get HDD parameters
   */
  void get_hdd_params();

  /**
   * @brief Get device name from mount point
   * @param [in] mount_point Mount point
   * @return Device name
   */
  std::string get_device_from_mount_point(const std::string & mount_point);

  /**
   * @brief Get HDD information from device name
   * @param [in] device Device name
   * @param [out] info HDD information
   * @return true on success, false on error
   */
  bool get_hdd_information(const std::string & device, hdd_reader_service::HddInformation & info);

  /**
   * @brief Update HDD connections
   */
  void update_hdd_connections();

  /**
   * @brief Unmount device
   * @param [in] device Device name
   * @return result of success or failure
   */
  int unmount_device(std::string & device);

  /**
   * @brief Timer callback
   */
  void on_timer();

  /**
   * @brief Update HDD information list
   */
  void update_hdd_info_list();

  /**
   * @brief Start HDD transfer measurement
   */
  void start_hdd_transfer_measurement();

  /**
   * @brief Update HDD statistics
   */
  void update_hdd_statistics();

  /**
   * @brief Get increment value of sysfs device stats per second
   * @param [in] cur_val Current value
   * @param [in] last_val last value
   * @param [in] duration_sec Duration in seconds
   * @return Increment value
   */
  static double get_increase_sysfs_device_stat_value_per_sec(
    uint64_t cur_val, uint64_t last_val, double duration_sec);

  /**
   * @brief Get statistics about the state of block device
   * @param [in] device Device name
   * @param [out] statistics Statistics of block device
   * @return 0 on success, otherwise error
   */
  int get_device_statistics(const std::string & device, DeviceStatistics & statistics);

  /**
   * @brief Connect to hdd-reader service
   * @return true on success, false on error
   */
  bool connect_service();

  /**
   * @brief Send data to hdd-reader service with parameter
   * @param [in] request Request to hdd-reader service
   * @param [in] parameter List of parameters
   * @return true on success, false on error
   */
  template <class T>
  bool send_data(hdd_reader_service::Request request, T & parameter);

  /**
   * @brief Receive data from hdd-reader service
   * @param [out] received Received data
   */
  template <class T>
  void receive_data(T & received);

  /**
   * @brief Close connection with hdd-reader service
   */
  void close_connection();

  /**
   * @brief Get column index of IP packet reassembles failed from `/proc/net/snmp`
   */
  void get_reassembles_failed_column_index();

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics
  rclcpp::TimerBase::SharedPtr timer_;   //!< @brief Timer to get HDD information from HddReader

  char hostname_[HOST_NAME_MAX + 1];  //!< @brief host name

  std::string socket_path_;             //!< @brief Path of UNIX domain socket
  boost::asio::io_service io_service_;  //!< @brief Core I/O functionality
  std::unique_ptr<local::stream_protocol::acceptor> acceptor_;      //!< @brief UNIX domain acceptor
  std::unique_ptr<local::stream_protocol::socket> socket_;          //!< @brief UNIX domain socket
  diagnostic_updater::DiagnosticStatusWrapper communication_diag_;  //!< @brief Communication errors

  std::map<std::string, HddParam> hdd_params_;       //!< @brief List of error and warning levels
  std::map<std::string, bool> hdd_connected_flags_;  //!< @brief List of HDD is connected
  hdd_reader_service::HddInformationList hdd_info_list_;      //!< @brief List of HDD information
  std::map<std::string, uint32_t> initial_recovered_errors_;  //!< @brief Initial recovered error
  std::map<std::string, HddStatistics> hdd_stats_;            //!< @brief List of HDD statistics
  rclcpp::Time last_hdd_stat_update_time_;  //!< @brief Last HDD statistics update time

  /**
   * @brief HDD connection status messages
   */
  const std::map<int, const char *> connection_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "not connected"}, {DiagStatus::ERROR, "unused"}};

  /**
   * @brief HDD status messages
   */
  const std::map<int, const char *> hdd_dicts_[static_cast<int>(CheckType::SIZE)] = {
    // Temperature
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "hot"}, {DiagStatus::ERROR, "critical hot"}},
    // Power On Hours
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "lifetime limit"}, {DiagStatus::ERROR, "unused"}},
    // Total data written
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "warranty period"}, {DiagStatus::ERROR, "unused"}},
    // Recovered error count
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high soft error rate"},
     {DiagStatus::ERROR, "unused"}},
    // Data rate of read
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high data rate of read"},
     {DiagStatus::ERROR, "unused"}},
    // Data rate of write
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high data rate of write"},
     {DiagStatus::ERROR, "unused"}},
    // IOPS of read
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high IOPS of read"},
     {DiagStatus::ERROR, "unused"}},
    // IOPS of write
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high IOPS of write"},
     {DiagStatus::ERROR, "unused"}},
  };

  /**
   * @brief HDD usage status messages
   */
  const std::map<int, const char *> usage_dict_ = {
    {DiagStatus::OK, "OK"},
    {DiagStatus::WARN, "low disk space"},
    {DiagStatus::ERROR, "very low disk space"}};
};

#endif  // SYSTEM_MONITOR__HDD_MONITOR__HDD_MONITOR_HPP_
