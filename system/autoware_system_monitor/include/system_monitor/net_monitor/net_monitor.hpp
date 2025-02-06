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
#include "system_monitor/traffic_reader/traffic_reader_common.hpp"

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
 * @brief Network information
 */
struct NetworkInfomation
{
  bool is_invalid;             //!< @brief Valid network to be checked
  int mtu_error_code;          //!< @brief Error code set by ioctl() with SIOCGIFMTU
  int ethtool_error_code;      //!< @brief Error code set by ioctl() with SIOCETHTOOL
  bool is_running;             //!< @brief Resource allocated flag
  std::string interface_name;  //!< @brief Interface name
  double speed;                //!< @brief Network capacity
  int mtu;                     //!< @brief MTU
  double rx_traffic;           //!< @brief Traffic received
  double tx_traffic;           //!< @brief Traffic transmitted
  double rx_usage;             //!< @brief Network capacity usage rate received
  double tx_usage;             //!< @brief Network capacity usage rate transmitted
  unsigned int rx_bytes;       //!< @brief Total bytes received
  unsigned int rx_errors;      //!< @brief Bad packets received
  unsigned int tx_bytes;       //!< @brief Total bytes transmitted
  unsigned int tx_errors;      //!< @brief Packet transmit problems
  unsigned int collisions;     //!< @brief Number of collisions during packet transmissions
};

/**
 * @brief Bytes information
 */
struct Bytes
{
  unsigned int rx_bytes;  //!< @brief Total bytes received
  unsigned int tx_bytes;  //!< @brief Total bytes transmitted
};

/**
 * @brief CRC errors information
 */
struct CrcErrors
{
  std::deque<unsigned int> errors_queue{};  //!< @brief queue that holds count of CRC errors
  unsigned int last_rx_crc_errors{0};  //!< @brief rx_crc_error at the time of the last monitoring
};

/**
 * @brief /proc/net/snmp information
 */
class NetSnmp
{
public:
  enum class Result {
    OK,
    CHECK_WARNING,
    READ_ERROR,
  };

  /**
   * @brief Constructor
   * @param [in] node node using this class.
   */
  explicit NetSnmp(rclcpp::Node * node);

  /**
   * @brief Constructor
   */
  NetSnmp() = delete;

  /**
   * @brief Copy constructor
   */
  NetSnmp(const NetSnmp &) = delete;

  /**
   * @brief Copy assignment operator
   */
  NetSnmp & operator=(const NetSnmp &) = delete;

  /**
   * @brief Move constructor
   */
  NetSnmp(const NetSnmp &&) = delete;

  /**
   * @brief Move assignment operator
   */
  NetSnmp & operator=(const NetSnmp &&) = delete;

  /**
   * @brief Set parameters for check
   * @param [in] check_duration the value for check_duration
   * @param [in] check_count the value for check_count
   */
  void set_check_parameters(unsigned int check_duration, unsigned int check_count);

  /**
   * @brief Find index in `/proc/net/snmp`
   * @param [in] protocol Protocol name (the first column string). e.g. "Ip:" or "Udp:"
   * @param [in] metrics Metrics name. e.g. "ReasmFails"
   */
  void find_index(const std::string & protocol, const std::string & metrics);

  /**
   * @brief Check metrics
   * @param [out] current_value the value read from snmp
   * @param [out] value_per_unit_time the increase of the value during the duration
   * @return the result of check
   */
  Result check_metrics(uint64_t & current_value, uint64_t & value_per_unit_time);

private:
  /**
   * @brief Read value from `/proc/net/snmp`
   * @param [in] index_row row in `/proc/net/snmp`
   * @param [in] index_col col in `/proc/net/snmp`
   * @param [out] output_value retrieved value
   * @return execution result
   */
  bool read_value_from_proc(
    unsigned int index_row, unsigned int index_col, uint64_t & output_value);

  rclcpp::Logger logger_;           //!< @brief logger gotten from user node
  unsigned int check_duration_;     //!< @brief check duration
  unsigned int check_count_;        //!< @brief check count threshold
  unsigned int index_row_;          //!< @brief index for the target metrics in /proc/net/snmp
  unsigned int index_col_;          //!< @brief index for the target metrics in /proc/net/snmp
  uint64_t current_value_;          //!< @brief the value read from snmp
  uint64_t last_value_;             //!< @brief the value read from snmp at the last monitoring
  uint64_t value_per_unit_time_;    //!< @brief the increase of the value during the duration
  std::deque<unsigned int> queue_;  //!< @brief queue that holds the delta of the value
};

namespace local = boost::asio::local;

class NetMonitor : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
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

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief Check network connection
   * @param [out] status diagnostic message passed directly to diagnostic publish calls
   */
  void check_connection(diagnostic_updater::DiagnosticStatusWrapper & status);

  /**
   * @brief Check network usage
   * @param [out] status diagnostic message passed directly to diagnostic publish calls
   */
  void check_usage(diagnostic_updater::DiagnosticStatusWrapper & status);

  /**
   * @brief Monitor network traffic
   * @param [out] status diagnostic message passed directly to diagnostic publish calls
   */
  void monitor_traffic(diagnostic_updater::DiagnosticStatusWrapper & status);

  /**
   * @brief Check CRC error
   * @param [out] status diagnostic message passed directly to diagnostic publish calls
   */
  void check_crc_error(diagnostic_updater::DiagnosticStatusWrapper & status);

  /**
   * @brief Check IP packet reassembles failed
   * @param [out] status diagnostic message passed directly to diagnostic publish calls
   */
  void check_reassembles_failed(diagnostic_updater::DiagnosticStatusWrapper & status);

  /**
   * @brief Check UDP buf errors
   * @param [out] status diagnostic message passed directly to diagnostic publish calls
   */
  void check_udp_buf_errors(diagnostic_updater::DiagnosticStatusWrapper & status);

  /**
   * @brief Timer callback
   */
  void on_timer();

  /**
   * @brief Determine if it is a supported network
   * @param [in] network Network infomation
   * @param [in] index Index of network infomation index
   * @param [out] status Diagnostic message passed directly to diagnostic publish calls
   * @param [out] error_message Error message
   */
  static void make_invalid_diagnostic_status(
    const NetworkInfomation & network, int index,
    diagnostic_updater::DiagnosticStatusWrapper & status, std::string & error_message);

  /**
   * @brief Update list of network information
   */
  void update_network_list();

  /**
   * @brief Update network information by using socket
   * @param [out] network Network information
   */
  void update_network_information_by_socket(NetworkInfomation & network);

  /**
   * @brief Update network information about MTU
   * @param [out] network Network information
   * @param [in] socket File descriptor to socket
   */
  static void update_mtu(NetworkInfomation & network, int socket);

  /**
   * @brief Update network information about network capacity
   * @param [out] network Network information
   * @param [in] socket File descriptor to socket
   */
  void update_network_capacity(NetworkInfomation & network, int socket);

  /**
   * @brief Update network information by using routing netlink stats
   * @param [out] network Network information
   * @param [in] data Pointer to routing netlink stats
   * @param [in] duration Time from previous measurement
   */
  void update_network_information_by_routing_netlink(
    NetworkInfomation & network, void * data, const rclcpp::Duration & duration);

  /**
   * @brief Update network information about network traffic
   * @param [out] network Network information
   * @param [in] stats Pointer to routing netlink stats
   * @param [in] duration Time from previous measurement
   */
  void update_traffic(
    NetworkInfomation & network, const struct rtnl_link_stats * stats,
    const rclcpp::Duration & duration);

  /**
   * @brief Update network information about CRC error
   * @param [out] network Network information
   * @param [in] stats Pointer to routing netlink stats
   */
  void update_crc_error(NetworkInfomation & network, const struct rtnl_link_stats * stats);

  /**
   * @brief Shutdown nl80211 object
   */
  void shutdown_nl80211();

  /**
   * @brief Send request to start nethogs
   */
  void send_start_nethogs_request();

  /**
   * @brief Send request to skip nethogs
   */
  void send_skip_nethogs_request();

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

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics
  rclcpp::TimerBase::SharedPtr timer_;   //!< @brief timer to get Network information

  char hostname_[HOST_NAME_MAX + 1];             //!< @brief host name
  std::map<std::string, Bytes> bytes_;           //!< @brief list of bytes
  rclcpp::Time last_update_time_;                //!< @brief last update time
  std::vector<std::string> device_params_;       //!< @brief list of devices
  NL80211 nl80211_;                              //!< @brief 802.11 netlink-based interface
  int getifaddrs_error_code_;                    //!< @brief Error code set by getifaddrs()
  std::vector<NetworkInfomation> network_list_;  //!< @brief List of Network information

  bool enable_traffic_monitor_;         //!< @brief enable nethogs
  std::string monitor_program_;         //!< @brief nethogs monitor program name
  std::string socket_path_;             //!< @brief Path of UNIX domain socket
  boost::asio::io_service io_service_;  //!< @brief Core I/O functionality
  std::unique_ptr<local::stream_protocol::acceptor> acceptor_;  //!< @brief UNIX domain acceptor
  std::unique_ptr<local::stream_protocol::socket> socket_;      //!< @brief UNIX domain socket

  std::map<std::string, CrcErrors> crc_errors_;  //!< @brief list of CRC errors
  unsigned int crc_error_check_duration_;        //!< @brief CRC error check duration
  unsigned int crc_error_count_threshold_;       //!< @brief CRC error count threshold

  NetSnmp reassembles_failed_info_;  //!< @brief information of IP packet reassembles failed
  NetSnmp udp_rcvbuf_errors_info_;   //!< @brief information of UDP rcv buf errors
  NetSnmp udp_sndbuf_errors_info_;   //!< @brief information of UDP snd buf errors

  /**
   * @brief Network connection status messages
   */
  const std::map<int, const char *> connection_messages_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "no such device"}, {DiagStatus::ERROR, "unused"}};

  /**
   * @brief Network usage status messages
   */
  const std::map<int, const char *> usage_messages_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high load"}, {DiagStatus::ERROR, "down"}};

  /**
   * @brief Interface name for loopback
   */
  const std::string loopback_interface_name_ = "lo";
};

#endif  // SYSTEM_MONITOR__NET_MONITOR__NET_MONITOR_HPP_
