#ifndef SYSTEM_MONITOR_NET_MONITOR_NL80211_H
#define SYSTEM_MONITOR_NET_MONITOR_NL80211_H
/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file nl80211.h
 * @brief 802.11 netlink-based interface class
 */

class NL80211
{
public:
  /**
   * @brief constructor
   */
  NL80211();

  /**
   * @brief initialize
   */
  void init(void);

  /**
   * @brief get bitrate
   * @param [in] ifa_name interface name
   * @return bitrate
   */
  float getBitrate(const char * ifa_name);

  /**
   * @brief shutdown
   */
  void shutdown(void);

  float bitrate_;  //!< @brief bitrate

private:
  bool
    initialized_;  //!< @brief Indicating whether initialization was completed successfully or not
  struct nl_sock * socket_;  //!< @brief Netlink socket
  int id_;                   //!< @brief Generic netlink family id
  struct nl_cb * cb_;        //!< @brief Callback handle
};

#endif  // SYSTEM_MONITOR_NET_MONITOR_NL80211_H
