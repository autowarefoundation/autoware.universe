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

#ifndef BLUETOOTH_MONITOR__SERVICE__L2PING_INTERFACE_HPP_
#define BLUETOOTH_MONITOR__SERVICE__L2PING_INTERFACE_HPP_

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>

#include <bluetooth/bluetooth.h>

#include <string>
#include <vector>

// 7634-7647 Unassigned
static constexpr int DEFAULT_PORT = 7640;
static constexpr int DEFAULT_DELAY = 1;
static constexpr int DEFAULT_TIMEOUT = 5;
static constexpr bool DEFAULT_VERIFY = false;
static constexpr float RTT_NO_WARN = 0.0f;

/**
 * @brief Configuration of L2ping
 */
struct L2pingConfig
{
  int delay{DEFAULT_DELAY};      //!< @brief Wait seconds between sending each packet
  int timeout{DEFAULT_TIMEOUT};  //!< @brief Wait timeout seconds for the response
  bool verify{DEFAULT_VERIFY};   //!< @brief Verify request and response payload
  float rtt_warn{RTT_NO_WARN};   //!< @brief RTT warning time

  /**
   * @brief Load or save data members.
   * @param [inout] ar Archive reference to load or save the serialized data members
   * @param [in] version Version for the archive
   * @note NOLINT syntax is needed since this is an interface to serialization and
   * used inside boost serialization.
   */
  template <typename archive>
  void serialize(archive & ar, const unsigned /*version*/)  // NOLINT(runtime/references)
  {
    ar & delay;
    ar & timeout;
    ar & verify;
    ar & rtt_warn;
  }
};

/**
 * @brief Configuration of L2ping service
 */
struct L2pingServiceConfig
{
  L2pingConfig l2ping{};               //!< @brief Configuration of L2ping
  std::vector<std::string> addresses;  //!< @brief List of bluetooth address

  /**
   * @brief Load or save data members.
   * @param [inout] ar Archive reference to load or save the serialized data members
   * @param [in] version Version for the archive
   * @note NOLINT syntax is needed since this is an interface to serialization and
   * used inside boost serialization.
   */
  template <typename archive>
  void serialize(archive & ar, const unsigned /*version*/)  // NOLINT(runtime/references)
  {
    ar & l2ping;
    ar & addresses;
  }
};

/**
 * @brief Status code of a device
 */
enum class StatusCode {
  OK = 0,
  RTT_WARNING = 1,
  VERIFY_ERROR = 2,
  LOST = 3,
  REJECTED = 4,
  FUNCTION_ERROR = 5,
};

/**
 * @brief L2ping status
 */
struct L2pingStatus
{
  StatusCode status_code;     //!< @brief Status code of a device
  std::string function_name;  //!< @brief Function name which error occurred
  int error_code;             //!< @brief Error number which is set by system calls

  std::string name;          //!< @brief Name of remote device
  std::string manufacturer;  //!< @brief Manufacturer name of remote device
  std::string address;       //!< @brief Bluetooth address
  float time_difference;     //!< @brief Time difference between sent and received
  int sent_packets;          //!< @brief Number of sent packets to remote device
  int received_packets;      //!< @brief Number of received packets from remote device

  /**
   * @brief Load or save data members.
   * @param [inout] ar Archive reference to load or save the serialized data members
   * @param [in] version Version for the archive
   * @note NOLINT syntax is needed since this is an interface to serialization and
   * used inside boost serialization.
   */
  template <typename archive>
  void serialize(archive & ar, const unsigned /*version*/)  // NOLINT(runtime/references)
  {
    ar & status_code;
    ar & function_name;
    ar & error_code;
    ar & name;
    ar & manufacturer;
    ar & address;
    ar & time_difference;
    ar & sent_packets;
    ar & received_packets;
  }
};

/**
 * @brief List of L2ping status
 */
typedef std::vector<L2pingStatus> L2pingStatusList;

#endif  // BLUETOOTH_MONITOR__SERVICE__L2PING_INTERFACE_HPP_
