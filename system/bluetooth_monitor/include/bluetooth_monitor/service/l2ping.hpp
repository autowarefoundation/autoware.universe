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

#ifndef BLUETOOTH_MONITOR__SERVICE__L2PING_HPP_
#define BLUETOOTH_MONITOR__SERVICE__L2PING_HPP_

#include "bluetooth_monitor/service/l2ping_interface.hpp"

#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/l2cap.h>

#include <string>
#include <thread>
#include <vector>

inline float tv2fl(const timeval & tv)
{
  return static_cast<float>(tv.tv_sec * 1000.0) + static_cast<float>(tv.tv_sec / 1000.0);
}

class L2ping
{
public:
  /**
   * @brief Constructor
   * @param [in]  address Bluetooth address of remote device
   * @param [in] config Configuration of L2ping
   */
  L2ping(const bdaddr_t & address, const L2pingConfig & config);

  /**
   * @brief Start ping thread
   */
  void run();

  /**
   * @brief Get information from remote device
   * @param [in] handle Handle to connection info
   * @return true on success, false on error
   */
  bool getDeviceInformation(uint16_t handle);

  /**
   * @brief Get status
   * @return Status
   */
  L2pingStatus getStatus() const;

  /**
   * @brief Get address of remote device
   * @return address of remote device
   */
  const std::string & getAddress() const;

protected:
  /**
   * @brief Thread loop
   */
  void thread();

  /**
   * @brief Connect to remote device
   * @return true on success, false on error
   */
  bool initialize();

  /**
   * @brief Shutdown
   */
  void shutdown();

  /**
   * @brief Ping to remote device
   * @return true on success, false on error
   */
  bool ping();

  /**
   * @brief Send echo command to remote device
   * @param [in] id id
   * @return true on success, false on error
   */
  bool sendEchoCommand(int id);

  /**
   * @brief Receive echo command from remote device
   * @return true on success, false on error
   */
  bool receiveEchoComand();

  /**
   * @brief Verify received command
   * @param [in] length Command length
   * @return true on success, false on error
   */
  bool verifyCommand(uint16_t length);

  /**
   * @brief Set function error data to inform ros2 node
   * @param [in] function_name Function name which error occurred
   * @param [in] error_code number which is set by system calls
   */
  void setFunctionError(const std::string & function_name, int error_code);

  /**
   * @brief Set status code
   * @param [in] code Status code
   */
  void setStatusCode(StatusCode code);

  bdaddr_t address_;     //!< @brief Bluetooth address of remote device
  L2pingConfig config_;  //!< @brief Configuration of L2ping
  int socket_;           //!< @brief Socket to L2CAP protocol
  std::thread thread_;   //!< @brief Thread to L2ping
  L2pingStatus status_;  //!< @brief L2ping status

  static constexpr int SIZE = 44;  //!< @brief The size of the data packets to be sent
  unsigned char send_buffer_[L2CAP_CMD_HDR_SIZE + SIZE];     //!< @brief buffer to send
  unsigned char receive_buffer_[L2CAP_CMD_HDR_SIZE + SIZE];  //!< @brief buffer to receive
};

#endif  // BLUETOOTH_MONITOR__SERVICE__L2PING_HPP_
