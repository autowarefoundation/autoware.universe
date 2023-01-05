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

#include <hdd_reader/ata.hpp>
#include <hdd_reader/hdd_reader_service.hpp>
#include <hdd_reader/nvme.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <syslog.h>

namespace hdd_reader_service
{

HddReaderService::HddReaderService(std::string socket_path) : socket_path_(std::move(socket_path))
{
}

bool HddReaderService::initialize()
{
  // Remove previous binding
  remove(socket_path_.c_str());

  local::stream_protocol::endpoint endpoint(socket_path_);
  acceptor_ = std::make_unique<local::stream_protocol::acceptor>(io_service_, endpoint);

  // Run I/O service processing loop
  boost::system::error_code error_code;
  io_service_.run(error_code);
  if (error_code) {
    syslog(LOG_ERR, "Failed to run I/O service. %s\n", error_code.message().c_str());
    return false;
  }

  // Give permission to other users to access to socket
  // NOLINTNEXTLINE [hicpp-signed-bitwise]
  if (chmod(socket_path_.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0) {
    syslog(LOG_ERR, "Failed to give permission to unix domain socket. %s\n", strerror(errno));
    return false;
  }

  return true;
}

void HddReaderService::shutdown() { io_service_.stop(); }

void HddReaderService::run()
{
  while (true) {
    socket_ = std::make_unique<local::stream_protocol::socket>(io_service_);

    // Accept a new connection
    boost::system::error_code error_code;
    acceptor_->accept(*socket_, error_code);

    if (error_code) {
      syslog(LOG_ERR, "Failed to accept new connection. %s\n", error_code.message().c_str());
      socket_->close();
      continue;
    }

    // Read data from socket
    char buffer[1024]{};
    socket_->read_some(boost::asio::buffer(buffer, sizeof(buffer)), error_code);

    if (error_code) {
      syslog(LOG_ERR, "Failed to read data from socket. %s\n", error_code.message().c_str());
      socket_->close();
      continue;
    }

    // Handle message
    handle_message(buffer);

    socket_->close();
  }
}

void HddReaderService::handle_message(const char * buffer)
{
  uint8_t request_id = Request::NONE;

  // Restore request from ros node
  std::istringstream in_stream(buffer);
  boost::archive::text_iarchive in(in_stream);

  try {
    in >> request_id;
  } catch (const std::exception & e) {
    syslog(LOG_ERR, "Failed to restore message. %s\n", e.what());
    return;
  }

  std::ostringstream out_stream;
  boost::archive::text_oarchive out(out_stream);

  // Set response code
  out << request_id;

  switch (request_id) {
    case Request::GET_HDD_INFORMATION:
      get_hdd_information(in, out);
      break;
    case Request::UNMOUNT_DEVICE:
      unmount_device(in, out);
      break;
    default:
      syslog(LOG_WARNING, "Unknown message. %d\n", request_id);
      return;
  }

  syslog(LOG_WARNING, "%s\n", out_stream.str().c_str());
  send_response(out_stream);
}

int HddReaderService::get_hdd_information(
  boost::archive::text_iarchive & in, boost::archive::text_oarchive & out)
{
  AttributeIdParameterList parameters;
  HddInformationList list;

  // Restore list of devices from ros node
  try {
    in & parameters;
  } catch (const std::exception & e) {
    syslog(LOG_ERR, "Failed to restore list of devices. %s\n", e.what());
    return EXIT_FAILURE;
  }

  for (auto & device : parameters) {
    HddInformation info{};
    const auto name = device.first.c_str();

    // Open a file
    int fd = open(name, O_RDONLY);
    if (fd < 0) {
      info.error_code = errno;
      syslog(LOG_ERR, "Failed to open a file. %s\n", strerror(errno));
      continue;
    }

    // AHCI device
    if (boost::starts_with(name, "/dev/sd")) {
      // Get HDD information from ATA drive
      info.error_code = Ata::get_ata(fd, device.second, info);
      // NVMe device
    } else if (boost::starts_with(name, "/dev/nvme")) {
      // Get HDD information from NVMe drive
      info.error_code = Nvme::get_nvme(fd, info);
    }

    // Close the file descriptor FD
    int ret = close(fd);
    if (ret < 0) {
      info.error_code = errno;
      syslog(LOG_ERR, "Failed to close the file descriptor FD. %s\n", strerror(errno));
    }

    list[name] = info;
  }

  out << list;

  return EXIT_SUCCESS;
}

int HddReaderService::unmount_device(
  boost::archive::text_iarchive & in, boost::archive::text_oarchive & out)
{
  std::string partition_name;

  // Restore partition name from ros node
  try {
    in & partition_name;
  } catch (const std::exception & e) {
    syslog(LOG_ERR, "Failed to restore partition name. %s\n", e.what());
    return EXIT_FAILURE;
  }

  boost::process::ipstream is_out;
  boost::process::ipstream is_err;

  boost::process::child c(
    "/bin/sh", "-c", fmt::format("umount -l {}", partition_name.c_str()),
    boost::process::std_out > is_out, boost::process::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    syslog(LOG_ERR, "Failed to umount. %s %s\n", partition_name.c_str(), strerror(c.exit_code()));
  }

  out << c.exit_code();
  return EXIT_SUCCESS;
}

void HddReaderService::send_response(const std::ostringstream & out_stream)
{
  // Write data to socket
  boost::system::error_code error_code;
  socket_->write_some(
    boost::asio::buffer(out_stream.str().c_str(), out_stream.str().length()), error_code);

  if (error_code) {
    syslog(LOG_ERR, "Failed to write data to socket. %s\n", error_code.message().c_str());
  }
}

}  // namespace hdd_reader_service
