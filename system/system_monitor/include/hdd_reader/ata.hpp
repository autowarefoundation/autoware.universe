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

#ifndef HDD_READER__ATA_HPP_
#define HDD_READER__ATA_HPP_

#include "hdd_reader/hdd_reader_common.hpp"

#include <boost/algorithm/string.hpp>

#include <scsi/sg.h>
#include <sys/ioctl.h>

#include <string>
#include <utility>

namespace hdd_reader_service
{

#pragma pack(1)

/**
 * @brief ATA PASS-THROUGH (12) command
 * @note For details please see the document below.
 * - ATA Command Pass-Through
 *   https://www.t10.org/ftp/t10/document.04/04-262r8.pdf
 */
struct AtaPassThrough12
{
  uint8_t operation_code;      //!< @brief OPERATION CODE (A1h)
  uint8_t reserved0 : 1;       //!< @brief Reserved
  uint8_t protocol : 4;        //!< @brief PROTOCOL
  uint8_t multiple_count : 3;  //!< @brief MULTIPLE_COUNT
  uint8_t t_length : 2;        //!< @brief T_LENGTH
  uint8_t byt_blok : 1;        //!< @brief BYT_BLOK
  uint8_t t_dir : 1;           //!< @brief T_DIR
  uint8_t reserved1 : 1;       //!< @brief Reserved
  uint8_t ck_cond : 1;         //!< @brief CK_COND
  uint8_t off_line : 2;        //!< @brief OFF_LINE
  uint8_t features;            //!< @brief FEATURES (0:7)
  uint8_t sector_count;        //!< @brief SECTOR_COUNT (0:7)
  uint8_t lba_low;             //!< @brief LBA_LOW (0:7)
  uint8_t lba_mid;             //!< @brief LBA_MID (0:7)
  uint8_t lbe_high;            //!< @brief LBE_HIGH (0:7)
  uint8_t device;              //!< @brief DEVICE
  uint8_t command;             //!< @brief COMMAND
  uint8_t reserved2;           //!< @brief Reserved
  uint8_t control;             //!< @brief CONTROL
};

/**
 * @brief Attribute Table Format
 * @note You can create an account Technical Committee T13(https://www.t13.org) and download
 * document.
 * - SMART Attribute Overview
 *   Search document number `e05171`.
 */
struct AttributeEntry
{
  uint8_t attribute_id;  //!< @brief Attribute ID
  //  Flags
  uint16_t warranty_ : 1;           //!< @brief Bit 0 - Warranty
  uint16_t offline_ : 1;            //!< @brief Bit 1 - Offline
  uint16_t performance_ : 1;        //!< @brief Bit 2 - Performance
  uint16_t error_rate_ : 1;         //!< @brief Bit 3 - Error rate
  uint16_t event_count_ : 1;        //!< @brief Bit 4 - Event count
  uint16_t self_preservation_ : 1;  //!< @brief Bit 5 - Self-preservation
  uint16_t reserved_ : 10;          //!< @brief Bits 6-15 - Reserved

  uint8_t current_value;        //!< @brief Current value
  uint8_t worst_value;          //!< @brief Worst value
  uint32_t data;                //!< @brief Data
  uint16_t attribute_specific;  //!< @brief Attribute-specific
  uint8_t threshold;            //!< @brief Threshold
};

/**
 * @brief Device SMART data structure
 * @note ou can create an account Technical Committee T13(https://www.t13.org) and download
 * document.
 * - ATA/ATAPI Command Set - 3 (ACS-3)
 *   Search document number `d2161` after creating your account.
 * - SMART Attribute Overview
 *   Search document number `e05171`.
 */
struct SmartData
{
  // Offset 0..361 X Vendor specific
  uint16_t smart_structure_version;    //!< @brief SMART structure version
  AttributeEntry attribute_entry[30];  //!< @brief Attribute entry 1 - 30
  // Offset 362 to 511
  uint8_t off_line_data_collection_status;      //!< @brief Off-line data collection status
  uint8_t self_test_execution_status_byte;      //!< @brief Self-test execution status byte
  uint16_t vendor_specific0;                    //!< @brief Vendor specific
  uint8_t vendor_specific1;                     //!< @brief Vendor specific
  uint8_t off_line_data_collection_capability;  //!< @brief Off-line data collection capability
  uint16_t smart_capability;                    //!< @brief SMART capability
  uint8_t error_logging_capability;             //!< @brief Error logging capability
  uint8_t vendor_specific2;                     //!< @brief Vendor specific
  uint8_t short_self_test_polling_time;     //!< @brief Short self-test polling time (in minutes)
  uint8_t extended_self_test_polling_time;  //!< @brief Extended self-test polling time in minutes
  uint8_t
    conveyance_self_test_polling_time;     //!< @brief Conveyance self-test polling time in minutes
  uint16_t                                 //!< @brief Extended self-test polling time
    extended_self_test_polling_time_word;  //!<   in minutes (word)
  uint8_t reserved_[9];                    //!< @brief Reserved
  uint8_t vendor_specific3[125];           //!< @brief Vendor specific
  uint8_t data_structure_checksum;         //!< @brief Data structure checksum
};

#pragma pack()

class Ata
{
public:
  /**
   * @brief Get IDENTIFY DEVICE for ATA drive
   * @param [in] fd File descriptor to device
   * @param [out] info HDD information
   * @return 0 on success, otherwise error
   * @note For details please see the documents below.
   * - ATA Command Pass-Through
   *   https://www.t10.org/ftp/t10/document.04/04-262r8.pdf
   * - ATA Command Set - 4  (ACS-4)
   *   You need to create your account of Technical Committee T13(https://www.t13.org)
   *   Search document number `di529` to download
   */
  static int get_ata_identify(int fd, HddInformation & info)
  {
    sg_io_hdr_t hdr{};
    AtaPassThrough12 ata{};
    unsigned char data[512]{};  // 256 words

    // Create a command descriptor block(CDB)
    ata.operation_code = 0xA1;  // ATA PASS-THROUGH (12) command
    ata.protocol = 0x4;         // PIO Data-In
    ata.t_dir = 0x1;            // from the ATA device to the application client
    ata.byt_blok = 0x1;         // the number of blocks specified in the T_LENGTH field
    ata.t_length = 0x2;         // length is specified in the SECTOR_COUNT field
    ata.sector_count = 0x01;    // 1 sector
    ata.command = 0xEC;         // IDENTIFY DEVICE

    // Create a control structure
    hdr.interface_id = 'S';                   // This must be set to 'S'
    hdr.dxfer_direction = SG_DXFER_FROM_DEV;  // a SCSI READ command
    hdr.cmd_len = sizeof(ata);         // length in bytes of the SCSI command that 'cmdp' points to
    hdr.cmdp = (unsigned char *)&ata;  // SCSI command to be executed
    hdr.dxfer_len = sizeof(data);      // number of bytes to be moved in the data transfer
    hdr.dxferp = data;                 // a pointer to user memory
    hdr.timeout = 1000;                // 1 second

    // Send SCSI command to device
    if (ioctl(fd, SG_IO, &hdr) < 0) {
      return errno;
    }

    // IDENTIFY DEVICE
    // Word 10..19 Serial number
    info.serial = std::string(reinterpret_cast<char *>(data) + 20, 20);
    swap_char(info.serial, 20);
    boost::trim(info.serial);

    // Word 27..46 Model number
    info.model = std::string(reinterpret_cast<char *>(data) + 54, 40);
    swap_char(info.model, 40);
    boost::trim(info.model);

    return EXIT_SUCCESS;
  }

  /**
   * @brief Get SMART DATA for ATA drive
   * @param [in] fd File descriptor to device
   * @param [in] parameter Device-specific attribute ID
   * @param [out] info HDD information
   * @return 0 on success, otherwise error
   * @note For details please see the documents below.
   * - ATA Command Pass-Through
   *   https://www.t10.org/ftp/t10/document.04/04-262r8.pdf
   * - You need to create your account of Technical Committee T13(https://www.t13.org)
   *   - ATA/ATAPI Command Set - 3 (ACS-3)
   *     Search document number `d2161` to download
   *   - SMART Attribute Overview
   *     Search document number `e05171` to download
   *   - SMART Attribute Annex
   *     Search document number `e05148` to download
   */
  static int get_ata_smart_data(
    int fd, const AttributeIdParameter & parameter, HddInformation & info)
  {
    sg_io_hdr_t hdr{};
    AtaPassThrough12 ata{};
    SmartData data{};

    // Create a command descriptor block(CDB)
    ata.operation_code = 0xA1;  // ATA PASS-THROUGH (12) command
    ata.protocol = 0x4;         // PIO Data-In
    ata.t_dir = 0x1;            // from the ATA device to the application client
    ata.byt_blok = 0x1;         // the number of blocks specified in the T_LENGTH field
    ata.t_length = 0x2;         // length is specified in the SECTOR_COUNT field
    ata.features = 0xD0;        // SMART READ DATA
    ata.sector_count = 0x01;    // 1 sector
    ata.lba_mid = 0x4F;         // Fixed
    ata.lbe_high = 0xC2;        // Fixed
    ata.command = 0xB0;         // SMART READ DATA

    // Create a control structure
    hdr.interface_id = 'S';                   // This must be set to 'S'
    hdr.dxfer_direction = SG_DXFER_FROM_DEV;  // a SCSI READ command
    hdr.cmd_len = sizeof(ata);         // length in bytes of the SCSI command that 'cmdp' points to
    hdr.cmdp = (unsigned char *)&ata;  // SCSI command to be executed
    hdr.dxfer_len = sizeof(data);      // number of bytes to be moved in the data transfer
    hdr.dxferp = &data;                // a pointer to user memory
    hdr.timeout = 1000;                // 1 second

    // Send SCSI command to device
    if (ioctl(fd, SG_IO, &hdr) < 0) {
      return errno;
    }

    info.is_valid_temp = false;
    info.is_valid_power_on_hours = false;
    info.is_valid_total_data_written = false;
    info.is_valid_recovered_error = false;

    // Retrieve S.M.A.R.T. Informations
    for (int i = 0; i < 30; ++i) {
      // Temperature - Device Internal
      if (data.attribute_entry[i].attribute_id == parameter.temperature_id) {
        info.temp = static_cast<uint8_t>(data.attribute_entry[i].data);
        info.is_valid_temp = true;
        // Power-on Hours Count
      } else if (data.attribute_entry[i].attribute_id == parameter.power_on_hours_id) {
        info.power_on_hours = data.attribute_entry[i].data;
        info.is_valid_power_on_hours = true;
        // Total LBAs Written
      } else if (data.attribute_entry[i].attribute_id == parameter.total_data_written_id) {
        info.total_data_written =
          (data.attribute_entry[i].data |
           (static_cast<uint64_t>(data.attribute_entry[i].attribute_specific) << 32));
        info.is_valid_total_data_written = true;
        // Hardware ECC Recovered
      } else if (data.attribute_entry[i].attribute_id == parameter.recovered_error_id) {
        info.recovered_error = data.attribute_entry[i].data;
        info.is_valid_recovered_error = true;
      }
    }

    return EXIT_SUCCESS;
  }

  /**
   * @brief Exchanges the values of 2 bytes
   * @param [inout] str A string reference to ATA string
   * @param [in] size Size of ATA string
   * @note Each pair of bytes in an ATA string is swapped.
   * FIRMWARE REVISION field example
   * Word Value
   * 23   6162h ("ba")
   * 24   6364h ("dc")
   * 25   6566h ("fe")
   * 26   6720h (" g")
   * -> "abcdefg "
   */
  static void swap_char(std::string & str, size_t size)
  {
    for (auto i = 0U; i < size; i += 2U) {
      std::swap(str[i], str[i + 1]);
    }
  }
};

}  // namespace hdd_reader_service

#endif  // HDD_READER__ATA_HPP_
