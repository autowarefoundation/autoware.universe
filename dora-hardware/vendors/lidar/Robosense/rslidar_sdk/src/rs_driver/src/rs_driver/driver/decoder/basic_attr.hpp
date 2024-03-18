/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <rs_driver/common/rs_common.hpp>

#include <fstream>
#include <cmath>
#include <algorithm>
#include <functional>
#include <chrono>
#include <mutex>

namespace robosense
{
namespace lidar
{

#pragma pack(push, 1)

typedef struct
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t ms;
  uint16_t us;
} RSTimestampYMD;

typedef struct 
{
  uint8_t sec[6];
  uint8_t ss[4];
} RSTimestampUTC;

typedef struct
{
  uint8_t tt[2];
} RSTemperature;

#pragma pack(pop)

#ifdef ENABLE_STAMP_WITH_LOCAL
inline long getTimezone(void)
{
  static long tzone = 0;
  static bool tzoneReady = false;

  if (!tzoneReady)
  {
#ifdef _MSC_VER
    _get_timezone(&tzone);
#else
    tzset();
    tzone = timezone;
#endif

    tzoneReady = true;
  }

  return tzone;
}
#endif

inline uint64_t parseTimeUTCWithUs(const RSTimestampUTC* tsUtc)
{
  // sec
  uint64_t sec = 0;
  for (int i = 0; i < 6; i++)
  {
    sec <<= 8;
    sec += tsUtc->sec[i];
  }

  // us
  uint64_t us = 0;
  for (int i = 0; i < 4; i++)
  {
    us <<= 8;
    us += tsUtc->ss[i]; 
  }

#ifdef ENABLE_STAMP_WITH_LOCAL
  sec -= getTimezone();
#endif

  return (sec * 1000000 + us);
}

inline void createTimeUTCWithUs(uint64_t us, RSTimestampUTC* tsUtc)
{
  uint64_t sec  = us / 1000000;
  uint64_t usec = us % 1000000;

#ifdef ENABLE_STAMP_WITH_LOCAL
  sec += getTimezone();
#endif

  for (int i = 5; i >= 0; i--)
  {
    tsUtc->sec[i] = sec & 0xFF;
    sec >>= 8;
  }

  for (int i = 3; i >= 0; i--)
  {
    tsUtc->ss[i] = usec & 0xFF;
    usec >>= 8;
  }
}

inline uint64_t parseTimeYMD(const RSTimestampYMD* tsYmd)
{
  std::tm stm;
  memset(&stm, 0, sizeof(stm));

  // since 2000 in robosense YMD, and since 1900 in struct tm
  stm.tm_year = tsYmd->year + (2000 - 1900); 
  // since 1 in robosense YMD, and since 0 in struct tm
  stm.tm_mon = tsYmd->month - 1; 
  // since 1 in both robosense YMD and struct tm
  stm.tm_mday = tsYmd->day; 
  stm.tm_hour = tsYmd->hour;
  stm.tm_min = tsYmd->minute;
  stm.tm_sec = tsYmd->second;
  time_t sec = std::mktime(&stm);

  uint64_t ms = ntohs(tsYmd->ms);
  uint64_t us = ntohs(tsYmd->us);

#if 0
  std::cout << "tm_year:" << stm.tm_year 
    << ", tm_mon:" << stm.tm_mon 
    << ", tm_day:" << stm.tm_mday
    << ", tm_hour:" << stm.tm_hour
    << ", tm_min:" << stm.tm_min
    << ", tm_sec:" << stm.tm_sec
    << ", ms:" << ms 
    << ", us:" << us 
    << std::endl;
#endif

#ifdef ENABLE_STAMP_WITH_LOCAL
  sec -= getTimezone();
#endif

  return (sec * 1000000 + ms * 1000 + us);
}

inline void createTimeYMD(uint64_t usec, RSTimestampYMD* tsYmd)
{
  uint64_t us = usec % 1000;
  uint64_t tot_ms = (usec - us) / 1000;

  uint64_t ms = tot_ms % 1000;
  uint64_t sec = tot_ms / 1000;

#ifdef ENABLE_STAMP_WITH_LOCAL
  sec += getTimezone();
#endif

  time_t t_sec = sec;

  std::tm* stm = localtime(&t_sec);

#if 0
  std::cout << "+ tm_year:" << stm->tm_year 
    << ", tm_mon:" << stm->tm_mon 
    << ", tm_day:" << stm->tm_mday
    << ", tm_hour:" << stm->tm_hour
    << ", tm_min:" << stm->tm_min
    << ", tm_sec:" << stm->tm_sec
    << ", ms:" << ms 
    << ", us:" << us 
    << std::endl;
#endif

  // since 2000 in robosense YMD, and since 1900 in struct tm
  tsYmd->year = stm->tm_year - (2000 - 1900); 
  // since 1 in robosense YMD, and since 0 in struct tm
  tsYmd->month = stm->tm_mon + 1; 
  // since 1 in both robosense YMD and struct tm
  tsYmd->day = stm->tm_mday;
  tsYmd->hour = stm->tm_hour;
  tsYmd->minute = stm->tm_min;
  tsYmd->second = stm->tm_sec;

  tsYmd->ms = htons((uint16_t)ms);
  tsYmd->us = htons((uint16_t)us);
}

inline uint64_t getTimeHost(void)
{
  std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
  std::chrono::system_clock::duration t_s = t.time_since_epoch();

  std::chrono::duration<uint64_t, std::ratio<1l, 1000000l>> t_us = 
    std::chrono::duration_cast<std::chrono::duration<uint64_t, std::ratio<1l, 1000000l>>>(t_s);
  return t_us.count();
}

inline int16_t parseTempInLe(const RSTemperature* tmp) // format of little endian
{
  // | lsb | padding | neg | msb |
  // |  5  |    3    |  1  |  7  | (in bits)
  uint8_t lsb = tmp->tt[0] >> 3;
  uint8_t neg = tmp->tt[1] & 0x80;
  uint8_t msb = tmp->tt[1] & 0x7F;

  int16_t t = ((uint16_t)msb << 5) + lsb;
  if (neg) t = -t;

  return t;
}

inline int16_t parseTempInBe(const RSTemperature* tmp) // format of big endian
{
  // | neg | msb | lsb | padding |
  // |  1  |  7  |  4  |    4    | (in bits)
  uint8_t neg = tmp->tt[0] & 0x80;
  uint8_t msb = tmp->tt[0] & 0x7F;
  uint8_t lsb = tmp->tt[1] >> 4;

  int16_t t = ((uint16_t)msb << 4) + lsb;
  if (neg) t = -t;

  return t;
}

inline uint32_t calcCrc32(const uint8_t *data, uint32_t len, 
    uint32_t startValue, bool isFirstCall)
{
  static const uint32_t crc32table[] =
  {
    0x00000000U, 0x77073096U, 0xee0e612cU, 0x990951baU, 0x076dc419U,
    0x706af48fU, 0xe963a535U, 0x9e6495a3U, 0x0edb8832U, 0x79dcb8a4U,
    0xe0d5e91eU, 0x97d2d988U, 0x09b64c2bU, 0x7eb17cbdU, 0xe7b82d07U,
    0x90bf1d91U, 0x1db71064U, 0x6ab020f2U, 0xf3b97148U, 0x84be41deU,
    0x1adad47dU, 0x6ddde4ebU, 0xf4d4b551U, 0x83d385c7U, 0x136c9856U,
    0x646ba8c0U, 0xfd62f97aU, 0x8a65c9ecU, 0x14015c4fU, 0x63066cd9U,
    0xfa0f3d63U, 0x8d080df5U, 0x3b6e20c8U, 0x4c69105eU, 0xd56041e4U,
    0xa2677172U, 0x3c03e4d1U, 0x4b04d447U, 0xd20d85fdU, 0xa50ab56bU,
    0x35b5a8faU, 0x42b2986cU, 0xdbbbc9d6U, 0xacbcf940U, 0x32d86ce3U,
    0x45df5c75U, 0xdcd60dcfU, 0xabd13d59U, 0x26d930acU, 0x51de003aU,
    0xc8d75180U, 0xbfd06116U, 0x21b4f4b5U, 0x56b3c423U, 0xcfba9599U,
    0xb8bda50fU, 0x2802b89eU, 0x5f058808U, 0xc60cd9b2U, 0xb10be924U,
    0x2f6f7c87U, 0x58684c11U, 0xc1611dabU, 0xb6662d3dU, 0x76dc4190U,
    0x01db7106U, 0x98d220bcU, 0xefd5102aU, 0x71b18589U, 0x06b6b51fU,
    0x9fbfe4a5U, 0xe8b8d433U, 0x7807c9a2U, 0x0f00f934U, 0x9609a88eU,
    0xe10e9818U, 0x7f6a0dbbU, 0x086d3d2dU, 0x91646c97U, 0xe6635c01U,
    0x6b6b51f4U, 0x1c6c6162U, 0x856530d8U, 0xf262004eU, 0x6c0695edU,
    0x1b01a57bU, 0x8208f4c1U, 0xf50fc457U, 0x65b0d9c6U, 0x12b7e950U,
    0x8bbeb8eaU, 0xfcb9887cU, 0x62dd1ddfU, 0x15da2d49U, 0x8cd37cf3U,
    0xfbd44c65U, 0x4db26158U, 0x3ab551ceU, 0xa3bc0074U, 0xd4bb30e2U,
    0x4adfa541U, 0x3dd895d7U, 0xa4d1c46dU, 0xd3d6f4fbU, 0x4369e96aU,
    0x346ed9fcU, 0xad678846U, 0xda60b8d0U, 0x44042d73U, 0x33031de5U,
    0xaa0a4c5fU, 0xdd0d7cc9U, 0x5005713cU, 0x270241aaU, 0xbe0b1010U,
    0xc90c2086U, 0x5768b525U, 0x206f85b3U, 0xb966d409U, 0xce61e49fU,
    0x5edef90eU, 0x29d9c998U, 0xb0d09822U, 0xc7d7a8b4U, 0x59b33d17U,
    0x2eb40d81U, 0xb7bd5c3bU, 0xc0ba6cadU, 0xedb88320U, 0x9abfb3b6U,
    0x03b6e20cU, 0x74b1d29aU, 0xead54739U, 0x9dd277afU, 0x04db2615U,
    0x73dc1683U, 0xe3630b12U, 0x94643b84U, 0x0d6d6a3eU, 0x7a6a5aa8U,
    0xe40ecf0bU, 0x9309ff9dU, 0x0a00ae27U, 0x7d079eb1U, 0xf00f9344U,
    0x8708a3d2U, 0x1e01f268U, 0x6906c2feU, 0xf762575dU, 0x806567cbU,
    0x196c3671U, 0x6e6b06e7U, 0xfed41b76U, 0x89d32be0U, 0x10da7a5aU,
    0x67dd4accU, 0xf9b9df6fU, 0x8ebeeff9U, 0x17b7be43U, 0x60b08ed5U,
    0xd6d6a3e8U, 0xa1d1937eU, 0x38d8c2c4U, 0x4fdff252U, 0xd1bb67f1U,
    0xa6bc5767U, 0x3fb506ddU, 0x48b2364bU, 0xd80d2bdaU, 0xaf0a1b4cU,
    0x36034af6U, 0x41047a60U, 0xdf60efc3U, 0xa867df55U, 0x316e8eefU,
    0x4669be79U, 0xcb61b38cU, 0xbc66831aU, 0x256fd2a0U, 0x5268e236U,
    0xcc0c7795U, 0xbb0b4703U, 0x220216b9U, 0x5505262fU, 0xc5ba3bbeU,
    0xb2bd0b28U, 0x2bb45a92U, 0x5cb36a04U, 0xc2d7ffa7U, 0xb5d0cf31U,
    0x2cd99e8bU, 0x5bdeae1dU, 0x9b64c2b0U, 0xec63f226U, 0x756aa39cU,
    0x026d930aU, 0x9c0906a9U, 0xeb0e363fU, 0x72076785U, 0x05005713U,
    0x95bf4a82U, 0xe2b87a14U, 0x7bb12baeU, 0x0cb61b38U, 0x92d28e9bU,
    0xe5d5be0dU, 0x7cdcefb7U, 0x0bdbdf21U, 0x86d3d2d4U, 0xf1d4e242U,
    0x68ddb3f8U, 0x1fda836eU, 0x81be16cdU, 0xf6b9265bU, 0x6fb077e1U,
    0x18b74777U, 0x88085ae6U, 0xff0f6a70U, 0x66063bcaU, 0x11010b5cU,
    0x8f659effU, 0xf862ae69U, 0x616bffd3U, 0x166ccf45U, 0xa00ae278U,
    0xd70dd2eeU, 0x4e048354U, 0x3903b3c2U, 0xa7672661U, 0xd06016f7U,
    0x4969474dU, 0x3e6e77dbU, 0xaed16a4aU, 0xd9d65adcU, 0x40df0b66U,
    0x37d83bf0U, 0xa9bcae53U, 0xdebb9ec5U, 0x47b2cf7fU, 0x30b5ffe9U,
    0xbdbdf21cU, 0xcabac28aU, 0x53b39330U, 0x24b4a3a6U, 0xbad03605U,
    0xcdd70693U, 0x54de5729U, 0x23d967bfU, 0xb3667a2eU, 0xc4614ab8U,
    0x5d681b02U, 0x2a6f2b94U, 0xb40bbe37U, 0xc30c8ea1U, 0x5a05df1bU,
    0x2d02ef8dU
  };

  if (isFirstCall)
  {
    startValue = 0xFFFFFFFFU;
  }
  else
  {
    /* undo the XOR on the start value */
    startValue ^= 0xFFFFFFFFU;

    /* The reflection of the initial value is not necessary here as we used
     * the "reflected" algorithm and reflected table values. */
  }

  /* Process all data byte-wise */
  while (len != 0U)
  {

    /* Process one byte of data */
    startValue = crc32table[((uint8_t)startValue) ^ *data] ^ (startValue >> 8U);
    /* Advance the pointer and decrease remaining bytes to calculate over
     * until all bytes in the buffer have been used as input */
    ++data;
    --len;
  } /* while (u32Length != 0U) */

  /* The reflection of the remainder is not necessary here as we used the
   * "reflected" algorithm and reflected table values. */
  startValue ^= 0xFFFFFFFFU; /* XOR crc value */

  return startValue;
}

inline bool isCrc32Correct(const uint8_t* pkt, size_t size)
{
  //
  // packet format
  //
  // | packet header + packet data | crc32    |  rolling_counter |
  // | n bytes                     | 4 bytes  |  2 bytes         |
  //
  uint32_t expected;
  expected = calcCrc32 (pkt, size - 6, 0/* ignored */, true);
  expected = calcCrc32 (pkt + size - 2, 2, expected, false);

  uint32_t actual = *(uint32_t*)(pkt + size - 6);
  actual = htonl(actual);

  return (expected == actual);
}

}  // namespace lidar
}  // namespace robosense
