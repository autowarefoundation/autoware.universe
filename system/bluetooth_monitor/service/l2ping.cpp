// Copyright 2022 Autoware Foundation
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

#include "bluetooth_monitor/service/l2ping.hpp"

#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <syslog.h>

#include <string>

static constexpr int IDENTIFIER = 200;  //!< @brief Identifier to match responses with requests

L2ping::L2ping(const bdaddr_t & address, const L2pingConfig & config)
: config_(config), socket_(-1), status_{}
{
  bacpy(&address_, &address);

  // Prepare string value fot log output
  char address_str[18]{};
  ba2str(&address_, address_str);
  status_.address = address_str;

  // Initialize buffer
  memset(send_buffer_, 0, sizeof(send_buffer_));
  for (int i = 0; i < SIZE; ++i) {
    send_buffer_[L2CAP_CMD_HDR_SIZE + i] = (i % 40) + 'A';
  }
}

bool L2ping::getDeviceInformation(uint16_t handle)
{
  // Retrieve the resource number of the Bluetooth adapter
  int dev_id = hci_get_route(&address_);
  if (dev_id < 0) {
    setFunctionError("hci_get_route", errno);
    syslog(
      LOG_ERR, "Device is not available. address = %s, %s", status_.address.c_str(),
      strerror(errno));
    return false;
  }

  // Opens a Bluetooth socket with the specified resource number
  int dev_sock = hci_open_dev(dev_id);
  if (dev_sock < 0) {
    setFunctionError("hci_open_dev", errno);
    syslog(
      LOG_ERR, "Failed to open HCI device. address = %s, %s", status_.address.c_str(),
      strerror(errno));
    return false;
  }

  // Get name of remote device
  char name[HCI_MAX_NAME_LENGTH]{};

  if (hci_read_remote_name(dev_sock, &address_, sizeof(name), name, 25000) == 0) {
    status_.name = name;
  }

  // Get version of remote device
  hci_version version{};

  if (hci_read_remote_version(dev_sock, handle, &version, 20000) == 0) {
    char * version_str = lmp_vertostr(version.lmp_ver);
    status_.manufacturer = bt_compidtostr(version.manufacturer);
    syslog(
      LOG_INFO,
      "Device Name: %s LMP Version: %s (0x%x) LMP Subversion: 0x%x Manufacturer: %s (%d)\n", name,
      version_str ? version_str : "n/a", version.lmp_ver, version.lmp_subver,
      bt_compidtostr(version.manufacturer), version.manufacturer);
    if (version_str) {bt_free(version_str);}
  }

  hci_close_dev(dev_sock);

  return true;
}

void L2ping::run()
{
  // Start thread loop
  thread_ = std::thread(&L2ping::thread, this);
}

void L2ping::thread()
{
  while (true) {
    // Connect to remote device
    if (initialize()) {
      // Ping to remote device in loop
      ping();
    }

    // Retry create socket
    shutdown();
    sleep(DEFAULT_DELAY);
  }

  shutdown();
}

bool L2ping::initialize()
{
  //  Create socket to L2CAP protocol
  socket_ = socket(PF_BLUETOOTH, SOCK_RAW, BTPROTO_L2CAP);

  if (socket_ < 0) {
    setFunctionError("socket", errno);
    syslog(
      LOG_ERR, "Failed to create socket for '%s'. %s\n", status_.address.c_str(), strerror(errno));
    return false;
  }

  // Register local address to connect to L2CAP
  sockaddr_l2 local_address{};
  local_address.l2_family = AF_BLUETOOTH;

  if (bind(socket_, (struct sockaddr *)&local_address, sizeof(local_address)) < 0) {
    setFunctionError("bind", errno);
    syslog(
      LOG_ERR, "Failed to bind socket for '%s'. %s\n", status_.address.c_str(), strerror(errno));
    return false;
  }

  // Connect to remote device
  sockaddr_l2 remote_address{};
  remote_address.l2_family = AF_BLUETOOTH;
  remote_address.l2_bdaddr = address_;

  if (connect(socket_, (struct sockaddr *)&remote_address, sizeof(remote_address)) < 0) {
    setFunctionError("connect", errno);
    syslog(LOG_ERR, "Failed to connect for '%s'. %s\n", status_.address.c_str(), strerror(errno));
    return false;
  }

  return true;
}

void L2ping::shutdown() {close(socket_);}

bool L2ping::ping()
{
  // Get local address
  sockaddr_l2 local_address{};
  socklen_t length = sizeof(local_address);

  if (getsockname(socket_, (struct sockaddr *)&local_address, &length) < 0) {
    setFunctionError("getsockname", errno);
    syslog(LOG_ERR, "Failed to get address. %s\n", strerror(errno));
    return false;
  }

  char host_address[18]{};
  ba2str(&local_address.l2_bdaddr, host_address);
  syslog(
    LOG_INFO, "Ping: %s from %s (data size %d) ...\n", status_.address.c_str(), host_address, SIZE);

  int id = IDENTIFIER;

  while (true) {
    timeval time_send{};
    timeval time_receive{};
    timeval time_diff{};

    gettimeofday(&time_send, NULL);

    // Send echo command
    if (!sendEchoCommand(id)) {
      return false;
    }

    // Send completed
    ++status_.sent_packets;

    l2cap_cmd_hdr * receive_command = reinterpret_cast<l2cap_cmd_hdr *>(receive_buffer_);

    // Wait for echo response
    while (true) {
      if (!receiveEchoComand()) {
        return false;
      }

      // Check received command
      receive_command->len = btohs(receive_command->len);

      // Check for our id
      if (receive_command->ident != id) {
        // Retry to receive our id
        continue;
      }
      // Check type
      if (receive_command->code == L2CAP_ECHO_RSP) {
        // Receive complete
        break;
      }
      // Echo command rejected
      if (receive_command->code == L2CAP_COMMAND_REJ) {
        setStatusCode(StatusCode::REJECTED);
        syslog(
          LOG_ERR, "Peer doesn't support echo packets for '%s'. %s\n", status_.address.c_str(),
          strerror(errno));
        return false;
      }
    }

    // Remote device responds correctly
    ++status_.received_packets;

    gettimeofday(&time_receive, NULL);
    timersub(&time_receive, &time_send, &time_diff);
    status_.time_difference = tv2fl(time_diff);

    bool failed = false;
    // If verify request and response payload
    if (config_.verify) {
      // Verify received command
      if (!verifyCommand(receive_command->len)) {
        failed = true;
        setStatusCode(StatusCode::VERIFY_ERROR);
      }
    }

    // If check RTT
    if (!failed && config_.rtt_warn > RTT_NO_WARN) {
      if (status_.time_difference >= config_.rtt_warn) {
        setStatusCode(StatusCode::RTT_WARNING);
      }
    }

    if (!failed) {setStatusCode(StatusCode::OK);}

    // Try next send
    sleep(config_.delay);
    if (++id > 254) {id = IDENTIFIER};
  }
}

bool L2ping::sendEchoCommand(int id)
{
  l2cap_cmd_hdr * send_command = reinterpret_cast<l2cap_cmd_hdr *>(send_buffer_);

  // Build command header
  send_command->ident = id;
  send_command->len = htobs(SIZE);
  send_command->code = L2CAP_ECHO_REQ;

  // Send echo command
  if (send(socket_, send_buffer_, sizeof(send_buffer_), 0) <= 0) {
    // Remote device is down
    setStatusCode(StatusCode::LOST);
    syslog(LOG_ERR, "Failed to send for '%s'. %s\n", status_.address.c_str(), strerror(errno));
    return false;
  }

  return true;
}

bool L2ping::receiveEchoComand()
{
  pollfd pf[1]{};
  int ret;

  pf[0].fd = socket_;
  pf[0].events = POLLIN;

  if ((ret = poll(pf, 1, config_.timeout * 1000)) < 0) {
    setFunctionError("poll", errno);
    syslog(LOG_ERR, "Failed to poll for '%s'. %s\n", status_.address.c_str(), strerror(errno));
    return false;
  }

  // Timeout
  if (!ret) {
    setStatusCode(StatusCode::LOST);
    syslog(LOG_ERR, "Polling timeout for '%s'.\n", status_.address.c_str());
    return false;
  }

  memset(receive_buffer_, 0, sizeof(receive_buffer_));

  if ((ret = recv(socket_, receive_buffer_, sizeof(receive_buffer_), 0)) < 0) {
    setStatusCode(StatusCode::LOST);
    syslog(LOG_ERR, "Failed to receive for '%s'. %s\n", status_.address.c_str(), strerror(errno));
    return false;
  }

  // Shutdown connection from device
  if (!ret) {
    setStatusCode(StatusCode::LOST);
    syslog(LOG_ERR, "Disconnected from '%s'. %s\n", status_.address.c_str(), strerror(errno));
    return false;
  }

  return true;
}

bool L2ping::verifyCommand(uint16_t length)
{
  // Check payload length
  if (length != SIZE) {
    syslog(LOG_ERR, "Received %d bytes, expected %d for %s", length, SIZE, status_.address.c_str());
    return false;
  }
  // Check payload
  if (memcmp(&send_buffer_[L2CAP_CMD_HDR_SIZE], &receive_buffer_[L2CAP_CMD_HDR_SIZE], SIZE)) {
    syslog(LOG_ERR, "Response payload different for %s.", status_.address.c_str());
    return false;
  }

  return true;
}

void L2ping::setFunctionError(const std::string & function_name, int error_code)
{
  // Build error data
  status_.status_code = StatusCode::FUNCTION_ERROR;
  status_.function_name = function_name;
  status_.error_code = error_code;
}

void L2ping::setStatusCode(StatusCode code)
{
  // Build error data
  status_.status_code = code;
  status_.function_name = {};
  status_.error_code = 0;
}

L2pingStatus L2ping::getStatus() const {return status_;}

const std::string & L2ping::getAddress() const {return status_.address;}
