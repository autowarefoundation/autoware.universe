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

/**
 * @file net_monitor.cpp
 * @brief Net monitor class
 */

#include "system_monitor/net_monitor/net_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <traffic_reader/traffic_reader_common.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/range/algorithm.hpp>
// #include <boost/algorithm/string.hpp>   // workaround for build errors

#include <fmt/format.h>
#include <ifaddrs.h>
#include <linux/ethtool.h>
#include <linux/if_link.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

NetMonitor::NetMonitor(const rclcpp::NodeOptions & options)
: Node("net_monitor", options),
  updater_(this),
  hostname_(),
  last_update_time_{0, 0, this->get_clock()->get_clock_type()},
  device_params_(
    declare_parameter<std::vector<std::string>>("devices", std::vector<std::string>())),
  getifaddrs_errno_(0),
  last_reassembles_failed_(0),
  monitor_program_(declare_parameter<std::string>("monitor_program", "greengrass")),
  crc_error_check_duration_(declare_parameter<int>("crc_error_check_duration", 1)),
  crc_error_count_threshold_(declare_parameter<int>("crc_error_count_threshold", 1)),
  reassembles_failed_check_duration_(
    declare_parameter<int>("reassembles_failed_check_duration", 1)),
  reassembles_failed_check_count_(declare_parameter<int>("reassembles_failed_check_count", 1)),
  reassembles_failed_column_index_(0),
  socket_path_(declare_parameter("socket_path", traffic_reader_service::socket_path))
{
  using namespace std::literals::chrono_literals;

  if (monitor_program_.empty()) {
    monitor_program_ = "*";
  }

  gethostname(hostname_, sizeof(hostname_));
  updater_.setHardwareID(hostname_);
  updater_.add("Network Usage", this, &NetMonitor::check_usage);
  updater_.add("Network Traffic", this, &NetMonitor::monitor_traffic);
  updater_.add("Network CRC Error", this, &NetMonitor::check_crc_error);
  updater_.add("IP Packet Reassembles Failed", this, &NetMonitor::check_reassembles_failed);

  nl80211_.init();

  search_reassembles_failed_column_index();

  // get Network information for the first time
  update_network_list();

  // Run I/O service processing loop
  boost::system::error_code error_code;
  io_service_.run(error_code);
  if (error_code) {
    RCLCPP_WARN(get_logger(), "Failed to run I/O service. %s", error_code.message().c_str());
  }

  // Send request to start nethogs
  send_start_nethogs_request();

  timer_ = rclcpp::create_timer(this, get_clock(), 1s, std::bind(&NetMonitor::on_timer, this));
}

NetMonitor::~NetMonitor() { shutdown_nl80211(); }

void NetMonitor::shutdown_nl80211() { nl80211_.shutdown(); }

void NetMonitor::on_timer() { update_network_list(); }

void NetMonitor::update_network_list()
{
  rclcpp::Duration duration = this->now() - last_update_time_;

  struct ifaddrs * interfaces = {};

  // Get network interfaces
  if (getifaddrs(&interfaces) < 0) {
    return;
  }

  for (const auto * interface = interfaces; interface; interface = interface->ifa_next) {
    // Skip no addr
    if (!interface->ifa_addr) {
      continue;
    }
    // Skip loopback
    if (interface->ifa_flags & IFF_LOOPBACK) {
      continue;
    }
    // Skip non AF_PACKET
    if (interface->ifa_addr->sa_family != AF_PACKET) {
      continue;
    }

    NetworkInfomation network{};
    network.interface_name = interface->ifa_name;
    network.is_running = (interface->ifa_flags & IFF_RUNNING);

    // Update network information using socket
    update_network_information_by_socket(network);

    // Update network information using routing netlink stats
    update_network_information_by_routing_netlink(network, interface->ifa_data, duration);

    network_list_[interface->ifa_name] = network;
  }

  freeifaddrs(interfaces);

  last_update_time_ = this->now();
}

void NetMonitor::update_network_information_by_socket(NetworkInfomation & network)
{
  // Get MTU information
  int fd = socket(AF_INET, SOCK_DGRAM, 0);

  // Update MTU information
  update_mtu(network, fd);

  // Update network capacity
  update_network_capacity(network, fd);

  close(fd);
}

void NetMonitor::update_mtu(NetworkInfomation & network, int fd)
{
  struct ifreq request = {};

  // NOLINTNEXTLINE [cppcoreguidelines-pro-type-union-access]
  strncpy(request.ifr_name, network.interface_name.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd, SIOCGIFMTU, &request) < 0) {
    network.mtu_errno = errno;
    return;
  }

  network.mtu = request.ifr_mtu;  // NOLINT [cppcoreguidelines-pro-type-union-access]
}

void NetMonitor::update_network_capacity(NetworkInfomation & network, int fd)
{
  struct ifreq request = {};
  struct ethtool_cmd ether_request = {};

  // NOLINTNEXTLINE [cppcoreguidelines-pro-type-union-access]
  strncpy(request.ifr_name, network.interface_name.c_str(), IFNAMSIZ - 1);
  request.ifr_data = (caddr_t)&ether_request;  // NOLINT [cppcoreguidelines-pro-type-cstyle-cast]

  ether_request.cmd = ETHTOOL_GSET;
  if (ioctl(fd, SIOCETHTOOL, &request) >= 0) {
    network.speed = ether_request.speed;
    return;
  }

  // Possibly wireless connection, get bitrate(MBit/s)
  int ret = nl80211_.getBitrate(network.interface_name.c_str());
  if (ret <= 0) {
    network.ethtool_errno = errno;
  } else {
    network.speed = ret;
  }
}

void NetMonitor::update_network_information_by_routing_netlink(
  NetworkInfomation & network, void * data, const rclcpp::Duration & duration)
{
  auto * stats = static_cast<struct rtnl_link_stats *>(data);

  update_traffic(network, stats, duration);

  update_crc_error(network, stats);
}

void NetMonitor::update_traffic(
  NetworkInfomation & network, const struct rtnl_link_stats * stats,
  const rclcpp::Duration & duration)
{
  network.rx_bytes = stats->rx_bytes;
  network.rx_errors = stats->rx_errors;
  network.tx_bytes = stats->tx_bytes;
  network.tx_errors = stats->tx_errors;
  network.collisions = stats->collisions;

  // Calculate traffic and usage if interface is entried in bytes
  const auto bytes_entry = bytes_.find(network.interface_name);
  if (bytes_entry != bytes_.end()) {
    network.rx_traffic =
      to_mbit(stats->rx_bytes - bytes_entry->second.rx_bytes) / duration.seconds();
    network.tx_traffic =
      to_mbit(stats->tx_bytes - bytes_entry->second.tx_bytes) / duration.seconds();
    network.rx_usage = network.rx_traffic / network.speed;
    network.tx_usage = network.tx_traffic / network.speed;
  }

  bytes_[network.interface_name].rx_bytes = stats->rx_bytes;
  bytes_[network.interface_name].tx_bytes = stats->tx_bytes;
}

void NetMonitor::update_crc_error(NetworkInfomation & network, const struct rtnl_link_stats * stats)
{
  // Get the count of CRC errors
  CrcErrors & crc_errors = crc_errors_[network.interface_name];
  crc_errors.errors_queue.push_back(stats->rx_crc_errors - crc_errors.last_rx_crc_errors);
  while (crc_errors.errors_queue.size() > crc_error_check_duration_) {
    crc_errors.errors_queue.pop_front();
  }
  crc_errors.last_rx_crc_errors = stats->rx_crc_errors;
}

void NetMonitor::check_usage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (!check_general_info(stat)) {
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  std::string error_str;
  std::vector<std::string> interface_names;

  for (const auto & network : network_list_) {
    if (!is_supported_network(network.second, index, stat, error_str)) {
      ++index;
      interface_names.push_back(network.second.interface_name);
      continue;
    }

    level = network.second.is_running ? DiagStatus::OK : DiagStatus::ERROR;

    stat.add(fmt::format("Network {}: status", index), usage_dict_.at(level));
    stat.add(fmt::format("Network {}: interface name", index), network.second.interface_name);
    stat.addf(fmt::format("Network {}: rx_usage", index), "%.2f%%", network.second.rx_usage * 1e+2);
    stat.addf(fmt::format("Network {}: tx_usage", index), "%.2f%%", network.second.tx_usage * 1e+2);
    stat.addf(
      fmt::format("Network {}: rx_traffic", index), "%.2f MBit/s", network.second.rx_traffic);
    stat.addf(
      fmt::format("Network {}: tx_traffic", index), "%.2f MBit/s", network.second.tx_traffic);
    stat.addf(fmt::format("Network {}: capacity", index), "%.1f MBit/s", network.second.speed);
    stat.add(fmt::format("Network {}: mtu", index), network.second.mtu);
    stat.add(fmt::format("Network {}: rx_bytes", index), network.second.rx_bytes);
    stat.add(fmt::format("Network {}: rx_errors", index), network.second.rx_errors);
    stat.add(fmt::format("Network {}: tx_bytes", index), network.second.tx_bytes);
    stat.add(fmt::format("Network {}: tx_errors", index), network.second.tx_errors);
    stat.add(fmt::format("Network {}: collisions", index), network.second.collisions);

    ++index;

    interface_names.push_back(network.second.interface_name);
  }

  // Check if specified device exists
  for (const auto & device : device_params_) {
    // Skip if all devices specified
    if (device == "*") {
      continue;
    }
    // Skip if device already appended
    if (boost::find(interface_names, device) != interface_names.end()) {
      continue;
    }

    stat.add(fmt::format("Network {}: status", index), "No Such Device");
    stat.add(fmt::format("Network {}: interface name", index), device);
    error_str = "no such device";
    ++index;
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(whole_level, usage_dict_.at(whole_level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void NetMonitor::check_crc_error(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (!check_general_info(stat)) {
    return;
  }

  int whole_level = DiagStatus::OK;
  int index = 0;
  std::string error_str;

  for (const auto & network : network_list_) {
    if (!is_supported_network(network.second, index, stat, error_str)) {
      ++index;
      continue;
    }

    CrcErrors & crc_errors = crc_errors_[network.second.interface_name];
    unsigned int unit_rx_crc_errors = 0;

    for (auto errors : crc_errors.errors_queue) {
      unit_rx_crc_errors += errors;
    }

    stat.add(fmt::format("Network {}: interface name", index), network.second.interface_name);
    stat.add(fmt::format("Network {}: total rx_crc_errors", index), crc_errors.last_rx_crc_errors);
    stat.add(fmt::format("Network {}: rx_crc_errors per unit time", index), unit_rx_crc_errors);

    if (unit_rx_crc_errors >= crc_error_count_threshold_) {
      whole_level = std::max(whole_level, static_cast<int>(DiagStatus::WARN));
      error_str = "CRC error";
    }

    ++index;
  }

  if (!error_str.empty()) {
    stat.summary(whole_level, error_str);
  } else {
    stat.summary(whole_level, "OK");
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

bool NetMonitor::check_general_info(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (device_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid device parameter");
    return false;
  }

  if (getifaddrs_errno_ != 0) {
    stat.summary(DiagStatus::ERROR, "getifaddrs error");
    stat.add("getifaddrs", strerror(getifaddrs_errno_));
    return false;
  }
  return true;
}

bool NetMonitor::is_supported_network(
  const NetworkInfomation & network, int index, diagnostic_updater::DiagnosticStatusWrapper & stat,
  std::string & error_str)
{
  // MTU information
  if (network.mtu_errno != 0) {
    if (network.mtu_errno == ENOTSUP) {
      stat.add(fmt::format("Network {}: status", index), "Not Supported");
    } else {
      stat.add(fmt::format("Network {}: status", index), "Error");
      error_str = "ioctl error";
    }

    stat.add(fmt::format("Network {}: interface name", index), network.interface_name);
    stat.add("ioctl(SIOCGIFMTU)", strerror(network.mtu_errno));
    return false;
  }

  // network capacity
  if (network.speed <= 0) {
    if (network.ethtool_errno == ENOTSUP) {
      stat.add(fmt::format("Network {}: status", index), "Not Supported");
    } else {
      stat.add(fmt::format("Network {}: status", index), "Error");
      error_str = "ioctl error";
    }

    stat.add(fmt::format("Network {}: interface name", index), network.interface_name);
    stat.add("ioctl(SIOCETHTOOL)", strerror(network.ethtool_errno));
    return false;
  }
  return true;
}

#include <boost/algorithm/string.hpp>  // workaround for build errors

void NetMonitor::monitor_traffic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  // Get result of nethogs
  traffic_reader_service::Result result;
  get_nethogs_result(result);

  // traffic_reader result to output
  if (result.error_code != EXIT_SUCCESS) {
    stat.summary(DiagStatus::ERROR, "traffic_reader error");
    stat.add("error", result.output);
  } else {
    stat.summary(DiagStatus::OK, "OK");

    if (result.output.empty()) {
      stat.add("nethogs: result", fmt::format("No data monitored: {}", monitor_program_));
    } else {
      std::stringstream lines{result.output};
      std::string line;
      std::vector<std::string> list;
      int idx = 0;
      while (std::getline(lines, line)) {
        if (line.empty()) {
          continue;
        }
        boost::split(list, line, boost::is_any_of("\t"), boost::token_compress_on);
        if (list.size() >= 3) {
          stat.add(fmt::format("nethogs {}: program", idx), list[3].c_str());
          stat.add(fmt::format("nethogs {}: sent (KB/s)", idx), list[1].c_str());
          stat.add(fmt::format("nethogs {}: received (KB/sec)", idx), list[2].c_str());
        } else {
          stat.add(fmt::format("nethogs {}: result", idx), line);
        }
        idx++;
      }
    }
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void NetMonitor::check_reassembles_failed(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  int whole_level = DiagStatus::OK;
  std::string error_str;
  uint64_t total_reassembles_failed = 0;
  uint64_t unit_reassembles_failed = 0;

  if (get_reassembles_failed(total_reassembles_failed)) {
    reassembles_failed_queue_.push_back(total_reassembles_failed - last_reassembles_failed_);
    while (reassembles_failed_queue_.size() > reassembles_failed_check_duration_) {
      reassembles_failed_queue_.pop_front();
    }

    for (auto reassembles_failed : reassembles_failed_queue_) {
      unit_reassembles_failed += reassembles_failed;
    }

    stat.add(fmt::format("total packet reassembles failed"), total_reassembles_failed);
    stat.add(fmt::format("packet reassembles failed per unit time"), unit_reassembles_failed);

    if (unit_reassembles_failed >= reassembles_failed_check_count_) {
      whole_level = std::max(whole_level, static_cast<int>(DiagStatus::WARN));
      error_str = "reassembles failed";
    }

    last_reassembles_failed_ = total_reassembles_failed;
  } else {
    reassembles_failed_queue_.push_back(0);
    whole_level = std::max(whole_level, static_cast<int>(DiagStatus::ERROR));
    error_str = "failed to read /proc/net/snmp";
  }

  if (!error_str.empty()) {
    stat.summary(whole_level, error_str);
  } else {
    stat.summary(whole_level, "OK");
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void NetMonitor::search_reassembles_failed_column_index()
{
  std::ifstream ifs("/proc/net/snmp");
  if (!ifs) {
    RCLCPP_WARN(get_logger(), "Failed to open /proc/net/snmp.");
    return;
  }

  // /proc/net/snmp
  // Ip: Forwarding DefaultTTL InReceives ... ReasmTimeout ReasmReqds ReasmOKs ReasmFails ...
  // Ip: 2          64         5636471397 ... 135          2303339    216166   270        ...
  std::string line;

  // Find column index of 'ReasmFails'
  if (!std::getline(ifs, line)) {
    RCLCPP_WARN(get_logger(), "Failed to get /proc/net/snmp first line.");
    return;
  }

  std::vector<std::string> title_list;
  boost::split(title_list, line, boost::is_space());

  if (title_list.empty()) {
    RCLCPP_WARN(get_logger(), "/proc/net/snmp first line is empty.");
    return;
  }
  if (title_list[0] != "Ip:") {
    RCLCPP_WARN(
      get_logger(), "/proc/net/snmp line title column is invalid. : %s", title_list[0].c_str());
    return;
  }

  int index = 0;
  for (auto itr = title_list.begin(); itr != title_list.end(); ++itr, ++index) {
    if (*itr == "ReasmFails") {
      reassembles_failed_column_index_ = index;
      break;
    }
  }
}

bool NetMonitor::get_reassembles_failed(uint64_t & reassembles_failed)
{
  if (reassembles_failed_column_index_ == 0) {
    RCLCPP_WARN(
      get_logger(), "reassembles failed column index is invalid. : %d",
      reassembles_failed_column_index_);
    return false;
  }

  std::ifstream ifs("/proc/net/snmp");
  if (!ifs) {
    RCLCPP_WARN(get_logger(), "Failed to open /proc/net/snmp.");
    return false;
  }

  std::string line;

  // Skip title row
  if (!std::getline(ifs, line)) {
    RCLCPP_WARN(get_logger(), "Failed to get /proc/net/snmp first line.");
    return false;
  }

  // Find a value of 'ReasmFails'
  if (!std::getline(ifs, line)) {
    RCLCPP_WARN(get_logger(), "Failed to get /proc/net/snmp second line.");
    return false;
  }

  std::vector<std::string> value_list;
  boost::split(value_list, line, boost::is_space());

  if (reassembles_failed_column_index_ >= value_list.size()) {
    RCLCPP_WARN(
      get_logger(),
      "There are not enough columns for reassembles failed column index. : columns=%d index=%d",
      static_cast<int>(value_list.size()), reassembles_failed_column_index_);
    return false;
  }

  reassembles_failed = std::stoull(value_list[reassembles_failed_column_index_]);

  return true;
}

void NetMonitor::send_start_nethogs_request()
{
  // Connect to boot/shutdown service
  if (!connect_service()) {
    close_connection();
    return;
  }

  int index = 0;
  std::string error_str;
  diagnostic_updater::DiagnosticStatusWrapper stat;
  std::vector<std::string> interface_names;

  for (const auto & network : network_list_) {
    if (!is_supported_network(network.second, index, stat, error_str)) {
      ++index;
      interface_names.push_back(network.second.interface_name);
      continue;
    }
    ++index;

    interface_names.push_back(network.second.interface_name);
  }

  // Send data to traffic-reader service
  send_data_with_parameters(
    traffic_reader_service::START_NETHOGS, interface_names, monitor_program_);

  // Close connection with traffic-reader service
  close_connection();
}

void NetMonitor::get_nethogs_result(traffic_reader_service::Result & result)
{
  // Connect to traffic-reader service
  if (!connect_service()) {
    close_connection();
    return;
  }

  // Send data to traffic-reader service
  if (!send_data(traffic_reader_service::Request::GET_RESULT)) {
    close_connection();
    return;
  }

  // Receive data from traffic-reader service
  receive_data(result);

  // Close connection with traffic-reader service
  close_connection();
}

bool NetMonitor::connect_service()
{
  local::stream_protocol::endpoint endpoint(socket_path_);
  socket_ = std::make_unique<local::stream_protocol::socket>(io_service_);

  // Connect socket
  boost::system::error_code error_code;
  socket_->connect(endpoint, error_code);

  if (error_code) {
    RCLCPP_ERROR(get_logger(), "Failed to connect socket. %s", error_code.message().c_str());
    return false;
  }

  return true;
}

bool NetMonitor::send_data(traffic_reader_service::Request request)
{
  std::ostringstream out_stream;
  boost::archive::text_oarchive archive(out_stream);
  archive & request;

  // Write data to socket
  boost::system::error_code error_code;
  socket_->write_some(
    boost::asio::buffer(out_stream.str().c_str(), out_stream.str().length()), error_code);

  if (error_code) {
    RCLCPP_ERROR(get_logger(), "Failed to write data to socket. %s", error_code.message().c_str());
    return false;
  }

  return true;
}

bool NetMonitor::send_data_with_parameters(
  traffic_reader_service::Request request, std::vector<std::string> & parameters,
  std::string & program_name)
{
  std::ostringstream out_stream;
  boost::archive::text_oarchive archive(out_stream);
  archive & request;
  archive & parameters;
  archive & program_name;

  // Write data to socket
  boost::system::error_code error_code;
  socket_->write_some(
    boost::asio::buffer(out_stream.str().c_str(), out_stream.str().length()), error_code);

  if (error_code) {
    RCLCPP_ERROR(get_logger(), "Failed to write data to socket. %s", error_code.message().c_str());
    return false;
  }

  return true;
}

void NetMonitor::receive_data(traffic_reader_service::Result & result)
{
  uint8_t request_id = traffic_reader_service::Request::NONE;

  // Read data from socket
  char buffer[10240]{};
  boost::system::error_code error_code;
  socket_->read_some(boost::asio::buffer(buffer, sizeof(buffer)), error_code);

  if (error_code) {
    RCLCPP_ERROR(
      get_logger(), "Failed to read data from socket. %s\n", error_code.message().c_str());
    return;
  }

  // Restore device status list
  try {
    std::istringstream in_stream(buffer);
    boost::archive::text_iarchive archive(in_stream);
    archive >> request_id;
    archive >> result;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to restore message. %s\n", e.what());
  }
}

void NetMonitor::close_connection()
{
  // Close socket
  socket_->close();
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(NetMonitor)
