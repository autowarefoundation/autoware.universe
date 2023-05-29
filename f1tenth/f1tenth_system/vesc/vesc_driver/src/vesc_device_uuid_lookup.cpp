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
#include <cassert>
#include <cmath>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <memory>

#include "vesc_driver/vesc_device_uuid_lookup.hpp"


namespace vesc_driver
{
using std::placeholders::_1;

VescDeviceLookup::VescDeviceLookup(std::string name)
: vesc_(
    std::string(),
    std::bind(&VescDeviceLookup::vescPacketCallback, this, _1),
    std::bind(&VescDeviceLookup::vescErrorCallback, this, _1)
),
  ready_(false),
  device_(name)
{
  try {
    vesc_.connect(device_);
    vesc_.requestFWVersion();
  } catch (SerialException e) {
    std::cerr << "VESC error on port " << device_ << std::endl << e.what() << std::endl;
    return;
  }
}

void VescDeviceLookup::close()
{
  vesc_.disconnect();
}

void VescDeviceLookup::vescPacketCallback(const std::shared_ptr<VescPacket const> & packet)
{
  if (packet->name() == "FWVersion") {
    std::shared_ptr<VescPacketFWVersion const> fw_version =
      std::dynamic_pointer_cast<VescPacketFWVersion const>(packet);

    const uint8_t * uuid = fw_version->uuid();

    hwname_ = fw_version->hwname();
    version_ = fw_version->fwMajor() + "." + fw_version->fwMinor();
    uuid_ += decode_uuid(uuid);
    ready_ = true;
  }
}

void VescDeviceLookup::vescErrorCallback(const std::string & error)
{
  error_ = error;
  ready_ = false;
}

bool VescDeviceLookup::isReady()
{
  return ready_;
}

const char * VescDeviceLookup::deviceUUID() const
{
  return uuid_.c_str();
}


const char * VescDeviceLookup::version() const
{
  return version_.c_str();
}

const char * VescDeviceLookup::hwname() const
{
  return hwname_.c_str();
}

}  // namespace vesc_driver
