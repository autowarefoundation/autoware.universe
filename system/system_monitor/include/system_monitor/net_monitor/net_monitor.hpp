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
 * @file net_monitor.h
 * @brief Net monitor class
 */

#ifndef SYSTEM_MONITOR__NET_MONITOR__NET_MONITOR_HPP_
#define SYSTEM_MONITOR__NET_MONITOR__NET_MONITOR_HPP_

#include "system_monitor/net_monitor/nl80211.hpp"
#include "traffic_reader/traffic_reader_common.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <boost/asio.hpp>

#include <climits>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

template <typename T>
constexpr auto to_mbit(T value)
{
  return (static_cast<double>(value) / 1000000 * 8);
}

/**
 * @brief Bytes information
 */
struct Bytes
{
  unsigned int rx_bytes{0};  //!< @brief total bytes received
  unsigned int tx_bytes{0};  //!< @brief total bytes transmitted
};

namespace local = boost::asio::local;

class NetMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  explicit NetMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief Destructor
   */
  ~NetMonitor() override;

  /**
   * @brief Copy constructor
   */
  NetMonitor(const NetMonitor &) = delete;

  /**
   * @brief Copy assignment operator
   */
  NetMonitor & operator=(const NetMonitor &) = delete;

  /**
   * @brief Move constructor
   */
  NetMonitor(const NetMonitor &&) = delete;

  /**
   * @brief Move assignment operator
   */
  NetMonitor & operator=(const NetMonitor &&) = delete;

  /**
   * @brief Shutdown nl80211 object
   */
  void shutdown_nl80211();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief check CPU usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void check_usage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief monitor traffic
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void monitor_traffic(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check CRC error
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void check_crc_error(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check IP packet reassembles failed
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void check_reassembles_failed(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief get wireless speed
   * @param [in] ifa_name interface name
   * @return wireless speed
   */
  float get_wireless_speed(const char * ifa_name);

  /**
   * @brief timer callback
   */
  void on_timer();

  /**
   * @brief update Network information list
   */
  void update_network_info_list();

  /**
   * @brief check NetMonitor General Infomation
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @return check result
   */
  bool check_general_info(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Send request to start nethogs
   */
  void send_start_nethogs_request();

  /**
   * @brief Get result of nethogs
   * @param [out] result result of nethogs
   */
  void get_nethogs_result(traffic_reader_service::Result & result);

  /**
   * @brief Connect to traffic-reader service
   * @return true on success, false on error
   */
  bool connect_service();

  /**
   * @brief Send data to traffic-reader service
   * @param [in] request Request to traffic-reader service
   * @return true on success, false on error
   */
  bool send_data(traffic_reader_service::Request request);

  /**
   * @brief Send data to traffic-reader service with parameters
   * @param [in] request Request to traffic-reader service
   * @param [in] parameters List of parameters
   * @param[in] program_name Filter by program name
   * @return true on success, false on error
   */
  bool send_data_with_parameters(
    traffic_reader_service::Request request, std::vector<std::string> & parameters,
    std::string & program_name);

  /**
   * @brief Receive data from traffic-reader service
   * @param [out] result Status from traffic-reader service
   */
  void receive_data(traffic_reader_service::Result & result);

  /**
   * @brief Close connection with traffic-reader service
   */
  void close_connection();

  /**
   * @brief Network information
   */
  struct NetworkInfo
  {
    int mtu_errno{0};              //!< @brief errno set by ioctl() with SIOCGIFMTU
    int ethtool_errno{0};          //!< @brief errno set by ioctl() with SIOCETHTOOL
    bool is_running{false};        //!< @brief resource allocated flag
    std::string interface_name{};  //!< @brief interface name
    double speed{0.0};             //!< @brief network capacity
    int mtu{0};                    //!< @brief MTU
    double rx_traffic{0.0};        //!< @brief traffic received
    double tx_traffic{0.0};        //!< @brief traffic transmitted
    double rx_usage{0.0};          //!< @brief network capacity usage rate received
    double tx_usage{0.0};          //!< @brief network capacity usage rate transmitted
    unsigned int rx_bytes{0};      //!< @brief total bytes received
    unsigned int rx_errors{0};     //!< @brief bad packets received
    unsigned int tx_bytes{0};      //!< @brief total bytes transmitted
    unsigned int tx_errors{0};     //!< @brief packet transmit problems
    unsigned int collisions{0};    //!< @brief number of collisions during packet transmissions
  };

  /**
   * @brief determine if it is a supported network
   * @param [in] net_info network infomation
   * @param [in] index index of network infomation index
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @param [out] error_str error string
   * @return result of determining whether it is a supported network
   */
  static bool is_supported_network(
    const NetworkInfo & net_info, int index, diagnostic_updater::DiagnosticStatusWrapper & stat,
    std::string & error_str);

  /**
   * @brief search column index of IP packet reassembles failed in /proc/net/snmp
   */
  void search_reassembles_failed_column_index();

  /**
   * @brief get IP packet reassembles failed
   * @param [out] reassembles_failed IP packet reassembles failed
   * @return execution result
   */
  bool get_reassembles_failed(uint64_t & reassembles_failed);

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics
  rclcpp::TimerBase::SharedPtr timer_;   //!< @brief timer to get Network information

  char hostname_[HOST_NAME_MAX + 1];        //!< @brief host name
  std::map<std::string, Bytes> bytes_;      //!< @brief list of bytes
  rclcpp::Time last_update_time_;           //!< @brief last update time
  std::vector<std::string> device_params_;  //!< @brief list of devices
  NL80211 nl80211_;                         //!< @brief 802.11 netlink-based interface
  int getifaddrs_errno_;                    //!< @brief errno set by getifaddrs()
  std::vector<NetworkInfo> net_info_list_;  //!< @brief list of Network information

  /**
   * @brief CRC errors information
   */
  struct CrcErrors
  {
    std::deque<unsigned int> errors_queue{};  //!< @brief queue that holds count of CRC errors
    unsigned int last_rx_crc_errors{0};  //!< @brief rx_crc_error at the time of the last monitoring
  };
  std::map<std::string, CrcErrors> crc_errors_;  //!< @brief list of CRC errors

  std::deque<unsigned int>
    reassembles_failed_queue_;  //!< @brief queue that holds count of IP packet reassembles failed
  uint64_t last_reassembles_failed_;  //!< @brief IP packet reassembles failed at the time of the
                                      //!< last monitoring

  std::string monitor_program_;             //!< @brief nethogs monitor program name
  unsigned int crc_error_check_duration_;   //!< @brief CRC error check duration
  unsigned int crc_error_count_threshold_;  //!< @brief CRC error count threshold
  unsigned int
    reassembles_failed_check_duration_;  //!< @brief IP packet reassembles failed check duration
  unsigned int
    reassembles_failed_check_count_;  //!< @brief IP packet reassembles failed check count threshold
  unsigned int reassembles_failed_column_index_;  //!< @brief column index of IP Reassembles failed
                                                  //!< in /proc/net/snmp

  std::string socket_path_;             //!< @brief Path of UNIX domain socket
  boost::asio::io_service io_service_;  //!< @brief Core I/O functionality
  std::unique_ptr<local::stream_protocol::acceptor> acceptor_;  //!< @brief UNIX domain acceptor
  std::unique_ptr<local::stream_protocol::socket> socket_;      //!< @brief UNIX domain socket

  /**
   * @brief Network usage status messages
   */
  const std::map<int, const char *> usage_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high load"}, {DiagStatus::ERROR, "down"}};
};

#endif  // SYSTEM_MONITOR__NET_MONITOR__NET_MONITOR_HPP_
