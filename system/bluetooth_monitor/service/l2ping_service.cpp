// Copyright 2022 The Autoware Contributors
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
// This code is derived from BlueZ with following copyrights.

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2000-2001  Qualcomm Incorporated
 *  Copyright (C) 2002-2003  Maxim Krasnyansky <maxk@qualcomm.com>
 *  Copyright (C) 2002-2010  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 */

#include "bluetooth_monitor/service/l2ping_service.hpp"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <bluetooth/rfcomm.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <syslog.h>

#include <memory>
#include <sstream>
#include <string>

static const bdaddr_t ANY_ADDRESS = {{0, 0, 0, 0, 0, 0}};

L2pingService::L2pingService(const int port) : port_(port), socket_(-1) {}

bool L2pingService::initialize()
{
  // Create a new socket
  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_ < 0) {
    syslog(LOG_ERR, "Failed to create a new socket. %s\n", strerror(errno));
    return false;
  }

  // Allow address reuse
  int ret = 0;
  int opt = 1;
  ret = setsockopt(
    socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char *>(&opt), (socklen_t)sizeof(opt));
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to set socket FD's option. %s\n", strerror(errno));
    return false;
  }

  // Give the socket FD the local address ADDR
  sockaddr_in address;
  memset(&address, 0, sizeof(sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  ret = bind(socket_, (struct sockaddr *)&address, sizeof(address));
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to give the socket FD the local address ADDR. %s\n", strerror(errno));
    return false;
  }

  // Prepare to accept connections on socket FD
  ret = listen(socket_, 5);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to prepare to accept connections on socket FD. %s\n", strerror(errno));
    return false;
  }

  return true;
}

void L2pingService::shutdown() { close(socket_); }

void L2pingService::run()
{
  sockaddr_in client;
  socklen_t len = sizeof(client);

  while (true) {
    // Await a connection on socket FD
    int new_sock = accept(socket_, reinterpret_cast<sockaddr *>(&client), &len);
    if (new_sock < 0) {
      syslog(
        LOG_ERR, "Failed to prepare to accept connections on socket FD. %s\n", strerror(errno));
      continue;
    }

    // Receive configuration of L2ping
    char buf[1024]{};
    int ret = recv(new_sock, buf, sizeof(buf) - 1, 0);
    if (ret < 0) {
      syslog(LOG_ERR, "Failed to receive. %s\n", strerror(errno));
      close(new_sock);
      continue;
    }
    // No data received
    if (ret == 0) {
      syslog(LOG_ERR, "No data received. %s\n", strerror(errno));
      close(new_sock);
      continue;
    }

    // Restore configuration of L2ping
    try {
      std::istringstream iss(buf);
      boost::archive::text_iarchive oa(iss);
      oa & config_;
    } catch (const std::exception & e) {
      syslog(LOG_ERR, "Exception occurred. %s\n", e.what());
      close(new_sock);
      continue;
    }

    // Now communication with ros2 node successful

    // Build device list to ping
    if (buildDeviceList()) {
      // Clear list
      status_list_.clear();
      // Build status list
      for (const auto & object : objects_) {
        status_list_.emplace_back(object->getStatus());
      }
    }

    // Inform ros2 node if error occurred
    std::ostringstream out_stream;
    boost::archive::text_oarchive archive(out_stream);
    archive << status_list_;
    //  Write N bytes of BUF to FD
    ret = write(new_sock, out_stream.str().c_str(), out_stream.str().length());
    if (ret < 0) {
      syslog(LOG_ERR, "Failed to write N bytes of BUF to FD. %s\n", strerror(errno));
    }

    close(new_sock);
  }
}

void L2pingService::setFunctionError(const std::string & function_name, int error_code)
{
  // Clear list
  status_list_.clear();

  // Set error data
  L2pingStatus status{};
  status.status_code = StatusCode::FUNCTION_ERROR;
  status.function_name = function_name;
  status.error_code = error_code;

  status_list_.emplace_back(status);
}

bool L2pingService::buildDeviceList()
{
  // Create socket to bluetooth host controller interface(HCI)
  int sock = socket(AF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI);

  if (sock < 0) {
    setFunctionError("socket", errno);
    syslog(LOG_ERR, "Failed to create socket. %s", strerror(errno));
    return false;
  }

  // Register local address to connect to HCI
  sockaddr_rc local_address{};

  local_address.rc_family = AF_BLUETOOTH;  // Bluetooth family
  local_address.rc_bdaddr = ANY_ADDRESS;   // Any address
  local_address.rc_channel = 1;            // Single channel

  if (bind(sock, (struct sockaddr *)&local_address, sizeof(local_address)) < 0) {
    setFunctionError("bind", errno);
    syslog(LOG_ERR, "Failed to bind socket. %s", strerror(errno));
    return false;
  }

  // Get list of HCI devices
  hci_dev_list_req hci_device_list{};
  hci_device_list.dev_num = HCI_MAX_DEV;

  if (ioctl(sock, HCIGETDEVLIST, &hci_device_list) < 0) {
    setFunctionError("HCIGETDEVLIST", errno);
    syslog(LOG_ERR, "Failed to get list of HCI devices. %s", strerror(errno));
    close(sock);
    return false;
  }

  // Get information of each HCI device
  hci_dev_req * hci_device = hci_device_list.dev_req;

  // Loop for HCI devices
  for (int i = 0; i < hci_device_list.dev_num; ++i, ++hci_device) {
    // Build device list to ping from connected devices
    if (!buildDeviceListFromConnectedDevices(sock, hci_device->dev_id)) {
      return false;
    }
  }

  close(sock);

  return true;
}

bool L2pingService::buildDeviceListFromConnectedDevices(int sock, uint16_t device_id)
{
  // Get informaton of HCI device
  hci_dev_info hci_device_info{};
  hci_device_info.dev_id = device_id;
  if (ioctl(sock, HCIGETDEVINFO, &hci_device_info) < 0) {
    setFunctionError("HCIGETDEVINFO", errno);
    syslog(
      LOG_ERR, "Failed to get informaton of HCI device. id = %d, %s", hci_device_info.dev_id,
      strerror(errno));
    return false;
  }

  // Get list of connected devices
  hci_conn_list_req connection_list_request{};
  connection_list_request.dev_id = hci_device_info.dev_id;
  connection_list_request.conn_num = HCI_MAX_DEV;

  if (ioctl(sock, HCIGETCONNLIST, &connection_list_request)) {
    setFunctionError("HCIGETCONNLIST", errno);
    syslog(
      LOG_ERR, "Failed to get connection list of HCI device. id = %d, %s", hci_device_info.dev_id,
      strerror(errno));
    return false;
  }

  // Allocate space for list of connected devices
  hci_conn_info * devices = new hci_conn_info[connection_list_request.conn_num];

  // Get information of each connected device
  hci_conn_info * connection_info = connection_list_request.conn_info;

  // Loop for connected devices
  for (int i = 0; i < connection_list_request.conn_num; ++i, ++connection_info) {
    char address_str[18]{};
    ba2str(&connection_info->bdaddr, address_str);

    // Skip if device not found and wild card not specified
    if (
      std::count(config_.addresses.begin(), config_.addresses.end(), address_str) == 0 &&
      std::count(config_.addresses.begin(), config_.addresses.end(), "*") == 0) {
      continue;
    }

    bool found = false;
    for (const auto & object : objects_) {
      // If device not registered
      if (object->getAddress() == address_str) {
        found = true;
        break;
      }
    }

    if (!found) {
      // Start ping thread
      objects_.emplace_back(std::make_unique<L2ping>(connection_info->bdaddr, config_.l2ping));
      objects_.back().get()->getDeviceInformation(connection_info->handle);
      objects_.back().get()->run();
    }
  }

  delete[] devices;

  return true;
}
