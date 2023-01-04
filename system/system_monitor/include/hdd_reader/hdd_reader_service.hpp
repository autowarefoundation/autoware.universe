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

#ifndef HDD_READER__HDD_READER_SERVICE_HPP_
#define HDD_READER__HDD_READER_SERVICE_HPP_

#include "hdd_reader/hdd_reader_common.hpp"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/asio.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace hdd_reader_service
{

namespace local = boost::asio::local;

class HddReaderService
{
public:
  /**
   * @brief Constructor
   * @param[in] socket_path Path of UNIX domain socket
   */
  explicit HddReaderService(std::string socket_path);

  /**
   * @brief Initialization
   * @return true on success, false on error
   */
  bool initialize();

  /**
   * @brief Shutdown
   */
  void shutdown();

  /**
   * @brief Main loop
   */
  void run();

protected:
  /**
   * @brief Handle message
   * @param[in] buffer Pointer to data received
   */
  void handle_message(const char * buffer);

  /**
   * @brief Get HDD information
   * @param [in] in boost::archive::text_iarchive object
   * @param [out] out boost::archive::text_oarchive object
   * @return 0 on success, otherwise error
   */
  int get_hdd_information(boost::archive::text_iarchive & in, boost::archive::text_oarchive & out);

  int get_ata(int fd, const AttributeIdParameter & parameter, HddInformation & info);

  int get_nvme(int fd, HddInformation & info);

  /**
   * @brief unmount device with lazy option
   * @param [in] boost::archive::text_iarchive object
   * @param [out] boost::archive::text_oarchive object
   * @return 0 on success, otherwise error
   */
  int unmount_device(boost::archive::text_iarchive & in, boost::archive::text_oarchive & out);

  void send_response(const std::ostringstream & out_stream);

  std::string socket_path_;             //!< @brief Path of UNIX domain socket
  boost::asio::io_service io_service_;  //!< @brief Core I/O functionality
  std::unique_ptr<local::stream_protocol::acceptor> acceptor_;  //!< @brief UNIX domain acceptor
  std::unique_ptr<local::stream_protocol::socket> socket_;      //!< @brief UNIX domain socket
};

}  // namespace hdd_reader_service

#endif  // HDD_READER__HDD_READER_SERVICE_HPP_
