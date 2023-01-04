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

#include <climits>
#include <map>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Error and warning temperature levels
 */
struct HddParam
{
  std::string part_device;                    //!< @brief Partition device
  std::string disk_device;                    //!< @brief Disk device
  float temp_warn{55.0};                      //!< @brief HDD temperature(DegC) to generate warning
  float temp_error{70.0};                     //!< @brief HDD temperature(DegC) to generate error
  int power_on_hours_warn{3000000};           //!< @brief HDD power on hours to generate warning
  uint64_t total_data_written_warn{4915200};  //!< @brief HDD total data written to generate warning
  float total_data_written_safety_factor{0.05};  //!< @brief Safety factor of HDD total data written
  int recovered_error_warn{1};        //!< @brief HDD recovered error count to generate warning
  int free_warn{5120};                //!< @brief HDD free space(MB) to generate warning
  int free_error{100};                //!< @brief HDD free space(MB) to generate error
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
 * @brief Statistics of sysfs device
 */
struct SysfsDevStat
{
  uint64_t rd_ios;      //!< @brief Number of read operations issued to the device
  uint64_t rd_sectors;  //!< @brief Number of sectors read
  uint64_t wr_ios;      //!< @brief Number of write operations issued to the device
  uint64_t wr_sectors;  //!< @brief Number of sectors written
};

/**
 * @brief Statistics of HDD
 */
struct HddStat
{
  std::string device;                //!< @brief Disk device
  std::string error_str;             //!< @brief Error string
  float read_data_rate_MBs;          //!< @brief Data rate of read (MB/s)
  float write_data_rate_MBs;         //!< @brief Data rate of write (MB/s)
  float read_iops;                   //!< @brief IOPS of read
  float write_iops;                  //!< @brief IOPS of write
  SysfsDevStat last_sysfs_dev_stat;  //!< @brief Last statistics of sysfs device
};

/**
 * @brief SMART information items to check
 */
enum class HddSmartInfoItem : uint32_t {
  TEMPERATURE = 0,
  POWER_ON_HOURS,
  TOTAL_DATA_WRITTEN,
  RECOVERED_ERROR,
  SIZE
};

/**
 * @brief HDD statistics items to check
 */
enum class HddStatItem : uint32_t {
  READ_DATA_RATE = 0,
  WRITE_DATA_RATE,
  READ_IOPS,
  WRITE_IOPS,
  SIZE
};

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
   * @param [in] item S.M.A.R.T information item to be checked
   */
  void check_smart(diagnostic_updater::DiagnosticStatusWrapper & stat, HddSmartInfoItem item);

  /**
   * @brief Check HDD usage
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_usage(diagnostic_updater::DiagnosticStatusWrapper & stat);

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
   * @param [in] item Statistic item to be checked
   */
  void check_statistics(diagnostic_updater::DiagnosticStatusWrapper & stat, HddStatItem item);

  /**
   * @brief Check HDD connection
   * @param [out] stat Diagnostic message passed directly to diagnostic publish calls
   */
  void check_connection(diagnostic_updater::DiagnosticStatusWrapper & stat);

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
   * @return device name
   */
  std::string get_device_from_mount_point(const std::string & mount_point);

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
  double get_increase_sysfs_device_stat_value_per_sec(
    uint64_t cur_val, uint64_t last_val, double duration_sec);

  /**
   * @brief Read stats for current whole device using /sys/block/ directory
   * @param [in] device Device name
   * @param [out] sysfs_dev_stat Statistics of sysfs device
   * @return result of success or failure
   */
  int read_sysfs_device_stat(const std::string & device, SysfsDevStat & sysfs_dev_stat);

  /**
   * @brief Connect to hdd-reader service
   * @return true on success, false on error
   */
  bool connect_service();

  /**
   * @brief Send data to hdd-reader service
   * @param [in] request Request to hdd-reader service
   * @return true on success, false on error
   */
  bool send_data(hdd_reader_service::Request request);

  /**
   * @brief Send data to hdd-reader service with parameters
   * @param [in] request Request to hdd-reader service
   * @param [in] parameters List of parameters
   * @return true on success, false on error
   */
  bool send_data_with_parameters(
    hdd_reader_service::Request request, hdd_reader_service::AttributeIdParameterList & parameters);

  /**
   * @brief Send data to hdd-reader service with parameters
   * @param [in] request Request to hdd-reader service
   * @param [in] parameter Device name
   * @return true on success, false on error
   */
  bool send_data_with_parameters(
    hdd_reader_service::Request request, std::string & parameter);

  /**
   * @brief Receive data from hdd-reader service
   * @param [out] list HDD information list
   */
  void receive_data(hdd_reader_service::HddInformationList & list);

  /**
   * @brief Receive data from hdd-reader service
   * @param [out] response Error code
   */
  void receive_data(int & response);

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
  std::unique_ptr<local::stream_protocol::acceptor> acceptor_;  //!< @brief UNIX domain acceptor
  std::unique_ptr<local::stream_protocol::socket> socket_;      //!< @brief UNIX domain socket

  std::map<std::string, HddParam> hdd_params_;  //!< @brief List of error and warning levels
  std::map<std::string, bool>
    hdd_connected_flags_;  //!< @brief List of flag whether HDD is connected
  std::map<std::string, uint32_t>
    initial_recovered_errors_;                //!< @brief List of initial recovered error count
  std::map<std::string, HddStat> hdd_stats_;  //!< @brief List of HDD statistics
  //!< @brief diagnostic of connection
  diagnostic_updater::DiagnosticStatusWrapper connect_diag_;
  hdd_reader_service::HddInformationList hdd_info_list_;  //!< @brief List of HDD information
  rclcpp::Time last_hdd_stat_update_time_;  //!< @brief Last HDD statistics update time

  /**
   * @brief HDD SMART status messages
   */
  const std::map<int, const char *> smart_dicts_[static_cast<uint32_t>(HddSmartInfoItem::SIZE)] = {
    // temperature
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "hot"}, {DiagStatus::ERROR, "critical hot"}},
    // power on hours
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "lifetime limit"}, {DiagStatus::ERROR, "unused"}},
    // total data written
    {{DiagStatus::OK, "OK"}, {DiagStatus::WARN, "warranty period"}, {DiagStatus::ERROR, "unused"}},
    // recovered error count
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high soft error rate"},
     {DiagStatus::ERROR, "unused"}},
  };

  /**
   * @brief HDD usage status messages
   */
  const std::map<int, const char *> usage_dict_ = {
    {DiagStatus::OK, "OK"},
    {DiagStatus::WARN, "low disk space"},
    {DiagStatus::ERROR, "very low disk space"}};

  /**
   * @brief HDD statistics status messages
   */
  const std::map<int, const char *> stat_dicts_[static_cast<uint32_t>(HddStatItem::SIZE)] = {
    // data rate of read
    {{DiagStatus::OK, "OK"},
     {DiagStatus::WARN, "high data rate of read"},
     {DiagStatus::ERROR, "unused"}},
    // data rate of write
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
   * @brief HDD connection status messages
   */
  const std::map<int, const char *> connection_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "not connected"}, {DiagStatus::ERROR, "unused"}};
};

#endif  // SYSTEM_MONITOR__HDD_MONITOR__HDD_MONITOR_HPP_
