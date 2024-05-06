/*
# Copyright  2021 Andrea Scipone
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#ifndef VESC_DRIVER__VESC_DEVICE_UUID_LOOKUP_HPP_
#define VESC_DRIVER__VESC_DEVICE_UUID_LOOKUP_HPP_


#include <string>
#include <memory>

#include "vesc_driver/datatypes.hpp"
#include "vesc_driver/vesc_interface.hpp"
#include "vesc_driver/vesc_packet.hpp"

namespace vesc_driver
{

class VescDeviceLookup
{
public:
  explicit VescDeviceLookup(std::string device);

  const char * deviceUUID() const;
  const char * version() const;
  const char * hwname() const;
  void close();
  bool isReady();

private:
  std::string device_;
  std::string uuid_;
  std::string version_;
  std::string hwname_;
  std::string error_;
  bool ready_;

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const std::shared_ptr<VescPacket const> & packet);
  void vescErrorCallback(const std::string & error);

  static std::string decode_uuid(const uint8_t * uuid)
  {
    char uuid_data[100] = "";
    std::string uuid_founded;
    snprintf(
      uuid_data, sizeof(uuid_data),
      "%02x%02x%02x-%02x%02x%02x-%02x%02x%02x-%02x%02x%02x",
      uuid[0], uuid[1], uuid[2],
      uuid[3], uuid[4], uuid[5],
      uuid[6], uuid[7], uuid[8],
      uuid[9], uuid[10], uuid[11]
    );
    return uuid_data;
  }
};
}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_DEVICE_UUID_LOOKUP_HPP_
