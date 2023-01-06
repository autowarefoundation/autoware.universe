// Copyright 2022 Tier IV, Inc.
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

#ifndef HDD_READER__HDD_READER_COMMON_HPP_
#define HDD_READER__HDD_READER_COMMON_HPP_

#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>

#include <map>
#include <string>

namespace hdd_reader_service
{

static constexpr char socket_path[] = "/tmp/hdd_reader";

/**
 * @brief Enumeration of Request ID to hdd_reader
 */
enum Request {
  NONE = 0,
  GET_HDD_INFORMATION,
  UNMOUNT_DEVICE,
};

/**
 * @brief Configure device-specific attribute ID of S.M.A.R.T. information
 */
struct AttributeIdParameter
{
  uint8_t temperature_id;         //!< @brief Temperature
  uint8_t power_on_hours_id;      //!< @brief Power on hours
  uint8_t total_data_written_id;  //!< @brief Total data written
  uint8_t recovered_error_id;     //!< @brief Recovered error

  /**
   * @brief Load or save data members.
   * @param [inout] ar archive reference to load or save the serialized data members
   * @param [in] version version for the archive
   */
  template <typename archive>
  void serialize(archive & ar, const unsigned /*version*/)  // NOLINT(runtime/references)
  {
    ar & temperature_id;
    ar & power_on_hours_id;
    ar & total_data_written_id;
    ar & recovered_error_id;
  }
};

/**
 * @brief HDD information
 */
struct HddInformation
{
  int error_code;      //!< @brief error code, 0 on success, otherwise error
  std::string model;   //!< @brief Model number
  std::string serial;  //!< @brief Serial number
  uint8_t temp;        //!< @brief temperature(DegC)
  // Lowest byte of the raw value contains the exact temperature value (Celsius degrees)
  // in S.M.A.R.T. information.
  uint64_t power_on_hours;           //!< @brief power on hours count
  uint64_t total_data_written;       //!< @brief total data written
  uint32_t recovered_error;          //!< @brief recovered error count
  bool is_valid_temp;                //!< @brief whether temp is valid value
  bool is_valid_power_on_hours;      //!< @brief whether power_on_hours_ is valid value
  bool is_valid_total_data_written;  //!< @brief whether total_data_written_ is valid value
  bool is_valid_recovered_error;     //!< @brief whether recovered_error_ is valid value

  /**
   * @brief Load or save data members.
   * @param [inout] ar archive reference to load or save the serialized data members
   * @param [in] version version for the archive
   * @note NOLINT syntax is needed since this is an interface to serialization and
   * used inside boost serialization.
   */
  template <typename archive>
  void serialize(archive & ar, const unsigned /*version*/)  // NOLINT(runtime/references)
  {
    ar & error_code;
    ar & model;
    ar & serial;
    ar & temp;
    ar & power_on_hours;
    ar & total_data_written;
    ar & recovered_error;
    ar & is_valid_temp;
    ar & is_valid_power_on_hours;
    ar & is_valid_total_data_written;
    ar & is_valid_recovered_error;
  }
};

/**
 * @brief Attribute ID list
 */
using AttributeIdParameterList = std::map<std::string, AttributeIdParameter>;

/**
 * @brief HDD information list
 */
using HddInformationList = std::map<std::string, HddInformation>;

}  // namespace hdd_reader_service

#endif  // HDD_READER__HDD_READER_COMMON_HPP_
