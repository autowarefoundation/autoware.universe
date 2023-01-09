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

#ifndef HDD_READER__NVME_HPP_
#define HDD_READER__NVME_HPP_

#include "hdd_reader/hdd_reader_common.hpp"

#include <boost/algorithm/string.hpp>

#include <linux/nvme_ioctl.h>
#include <sys/ioctl.h>
#include <syslog.h>

#include <string>

namespace hdd_reader_service
{

class Nvme
{
public:
  /**
   * @brief Get HDD information from NVMe drive
   * @param [in] fd File descriptor to device
   * @param [out] info HDD information
   * @return 0 on success, otherwise error
   */
  static int get_nvme(int fd, HddInformation & info)
  {
    // Get Identify for NVMe drive
    int ret = get_nvme_identify(fd, info);
    if (ret != EXIT_SUCCESS) {
      syslog(LOG_ERR, "Failed to get Identify for NVMe drive. %s\n", strerror(ret));
      return ret;
    }
    // Get SMART / Health Information for NVMe drive
    ret = get_nvme_smart_data(fd, info);
    if (ret != EXIT_SUCCESS) {
      syslog(LOG_ERR, "Failed to get SMART/Health Information for NVMe drive. %s\n", strerror(ret));
    }
    return ret;
  }

  /**
   * @brief Get Identify for NVMe drive
   * @param [in] fd File descriptor to device
   * @param [out] info HDD information
   * @return 0 on success, otherwise error
   * @note For details please see the document below.
   * - NVM Express 1.2b
   *   https://www.nvmexpress.org/wp-content/uploads/NVM_Express_1_2b_Gold_20160603.pdf
   */
  static int get_nvme_identify(int fd, HddInformation & info)
  {
    nvme_admin_cmd cmd{};
    char data[4096]{};  // Fixed size for Identify command

    // The Identify command returns a data buffer that describes information about the NVM subsystem
    cmd.opcode = 0x06;  // Identify
    // NOLINTNEXTLINE [cppcoreguidelines-pro-type-cstyle-cast]
    cmd.addr = (uint64_t)data;    // memory address of data
    cmd.data_len = sizeof(data);  // length of data
    cmd.cdw10 = 0x01;             // Identify Controller data structure

    // Send Admin Command to device
    if (ioctl(fd, NVME_IOCTL_ADMIN_CMD, &cmd) < 0) {
      return errno;
    }

    // Identify Controller Data Structure
    std::string identify = std::string(data, sizeof(data));
    // Bytes 23:04 Serial Number (SN)
    info.serial = identify.substr(4, 20);
    boost::trim(info.serial);

    // Bytes 63:24 Model Number (MN)
    info.model = identify.substr(24, 40);
    boost::trim(info.model);

    return EXIT_SUCCESS;
  }

  /**
   * @brief Get SMART / Health Information for NVMe drive
   * @param [in] fd File descriptor to device
   * @param [out] info HDD information
   * @return 0 on success, otherwise error
   * @note For details please see the document below.
   * - NVM Express 1.2b
   *   https://www.nvmexpress.org/wp-content/uploads/NVM_Express_1_2b_Gold_20160603.pdf
   */
  static int get_nvme_smart_data(int fd, HddInformation & info)
  {
    nvme_admin_cmd cmd{};
    unsigned char data[144]{};  // 36 Dword (get byte 0 to 143)

    // The Get Log Page command returns a data buffer containing the log page requested
    cmd.opcode = 0x02;      // Get Log Page
    cmd.nsid = 0xFFFFFFFF;  // Global log page
    // NOLINTNEXTLINE [cppcoreguidelines-pro-type-cstyle-cast]
    cmd.addr = (uint64_t)data;    // memory address of data
    cmd.data_len = sizeof(data);  // length of data
    cmd.cdw10 = 0x00240002;       // Bit 27:16 Number of Dwords (NUMD) = 024h (36 Dword)
                                  // - Minimum necessary size to obtain S.M.A.R.T. informations
                                  // Bit 07:00 = 02h (SMART / Health Information)

    // Send Admin Command to device
    if (ioctl(fd, NVME_IOCTL_ADMIN_CMD, &cmd) < 0) {
      return errno;
    }

    // Bytes 2:1 Composite Temperature
    // Convert kelvin to celsius
    unsigned int temperature = ((data[2] << 8u) | data[1]) - 273;
    info.is_valid_temp = true;
    info.temp = static_cast<uint8_t>(temperature);

    // Bytes 63:48 Data Units Written
    // This value is reported in thousands
    // (i.e., a value of 1 corresponds to 1,000 units of 512 bytes written)
    // and is rounded up
    // (e.g., one indicates that the number of 512 byte data units written
    // is from 1 to 1,000, three indicates that the number of 512 byte data
    // units written is from 2,001 to 3,000)
    info.is_valid_total_data_written = true;
    info.total_data_written = *(reinterpret_cast<uint64_t *>(&data[48]));

    // Bytes 143:128 Power On Hours
    info.is_valid_power_on_hours = true;
    info.power_on_hours = *(reinterpret_cast<uint64_t *>(&data[128]));

    // NVMe S.M.A.R.T has no information of recovered error count
    info.is_valid_recovered_error = false;

    return EXIT_SUCCESS;
  }
};

}  // namespace hdd_reader_service

#endif  // HDD_READER__NVME_HPP_
