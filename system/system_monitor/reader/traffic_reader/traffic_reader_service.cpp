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

/**
 * @file traffic_reader_service.cpp
 * @brief traffic information read class
 */

#include <traffic_reader/traffic_reader_service.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <syslog.h>

namespace bp = boost::process;

namespace traffic_reader_service
{

TrafficReaderService::TrafficReaderService(const std::string & socket_path)
: socket_path_(socket_path), socket_(-1), connection_(-1), stop_(false)
{
}

bool TrafficReaderService::initialize()
{
  // Remove previous binding
  remove(socket_path_.c_str());

  // Create a new socket
  socket_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (socket_ < 0) {
    syslog(LOG_ERR, "Failed to create a new socket. %s\n", strerror(errno));
    return false;
  }

  sockaddr_un addr = {};
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  // Give the socket FD the unix domain socket
  int ret = bind(socket_, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to give the socket FD the unix domain socket. %s\n", strerror(errno));
    shutdown();
    return false;
  }

  // Give permission to other users to access to socket
  if (chmod(socket_path_.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0) {  // NOLINT
                                                                        // [hicpp-signed-bitwise]
    syslog(LOG_ERR, "Failed to give permission to unix domain socket. %s\n", strerror(errno));
    return false;
  }

  // Prepare to accept connections on socket FD
  ret = listen(socket_, 5);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to prepare to accept connections on socket FD. %s\n", strerror(errno));
    shutdown();
    return false;
  }

  return true;
}

void TrafficReaderService::shutdown()
{
  close(socket_);
  socket_ = -1;
}

void TrafficReaderService::run()
{
  sockaddr_un client = {};
  socklen_t len = sizeof(client);

  while (true) {
    // Await a connection on socket FD
    connection_ = accept(socket_, reinterpret_cast<sockaddr *>(&client), &len);
    if (connection_ < 0) {
      syslog(
        LOG_ERR, "Failed to prepare to accept connections on socket FD. %s\n", strerror(errno));
      close(connection_);
      continue;
    }

    // Receive messages from a socket
    char buffer[1024]{};
    ssize_t ret = recv(connection_, buffer, sizeof(buffer), 0);
    if (ret < 0) {
      syslog(LOG_ERR, "Failed to recv. %s\n", strerror(errno));
      close(connection_);
      continue;
    }
    // No data received
    if (ret == 0) {
      syslog(LOG_ERR, "No data received.\n");
      close(connection_);
      continue;
    }

    // Handle message
    handle_message(buffer);

    close(connection_);
  }
}

void TrafficReaderService::handle_message(const char * buffer)
{
  uint8_t request_id = Request::NONE;

  // Restore request from ros node
  std::istringstream in_stream(buffer);
  boost::archive::text_iarchive archive(in_stream);

  try {
    archive >> request_id;
  } catch (const std::exception & e) {
    syslog(LOG_ERR, "Failed to restore message. %s\n", e.what());
    return;
  }

  switch (request_id) {
    case Request::START_NETHOGS:
      start_nethogs(archive);
      break;
    case Request::GET_RESULT:
      get_result();
      break;
    default:
      syslog(LOG_WARNING, "Unknown message. %d\n", request_id);
      break;
  }
}

void TrafficReaderService::start_nethogs(boost::archive::text_iarchive & archive)
{
  syslog(LOG_INFO, "Starting nethogs...\n");

  // Restore list of devices from ros node
  try {
    archive >> devices_;
    archive >> program_name_;
  } catch (const std::exception & e) {
    syslog(LOG_ERR, "Failed to restore optional commands. %s\n", e.what());
    devices_.clear();
  }

  // Stop thread if already running
  if (thread_.joinable()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    thread_.join();
  }

  // Run nethogs
  stop_ = false;
  thread_ = std::thread(&TrafficReaderService::execute_nethogs, this);
}

void TrafficReaderService::get_result()
{
  // Inform ros node
  std::ostringstream out_stream;
  boost::archive::text_oarchive archive(out_stream);
  archive & Request::GET_RESULT;
  archive & result_;

  //  Write N bytes of BUF to FD
  ssize_t ret = write(connection_, out_stream.str().c_str(), out_stream.str().length());
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to write N bytes of BUF to FD. %s\n", strerror(errno));
  }
}

/**
 * @brief thread function to execute nethogs
 */
void TrafficReaderService::execute_nethogs()
{
  std::stringstream command;
  command << "nethogs -t";

  for (const auto & device : devices_) {
    command << " " << device;
  }

  syslog(LOG_INFO, "%s\n", command.str().c_str());

  bp::ipstream is_out;
  bp::ipstream is_err;
  std::string line;

  boost::process::child c(command.str(), bp::std_out > is_out, bp::std_err > is_err);

  // Output format of `nethogs -t [device(s)]`
  //
  // Refreshing:
  // [PROGRAM/PID/UID SENT RECEIVED]
  // usr/share/code/code/3102772/1000 0.565234 1.06855
  // ...
  // unknown TCP/0/0 0 0
  while (std::getline(is_out, line)) {
    // Exit loop when stop is requested
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_) break;

    // Skip if line is empty
    if (line.empty()) {
      continue;
    }
    // Start of output
    if (line == "Refreshing:") {
      result_.error_code = EXIT_SUCCESS;
      result_.output.clear();
      continue;
    }
    // Skip if no PID
    if (line.find("/0/0\t") != std::string::npos) {
      continue;
    }
    // End of output
    if (line == "unknown TCP/0/0\t0\t0") {
      continue;
    }

    std::string command_line = get_command_line(line);
    if (!command_line.empty()) {
      // Add nethogs output and command line
      if (program_name_ == "*" || command_line.find(program_name_) != std::string::npos) {
        result_.output.append(fmt::format("{}\t{}\n", line, command_line));
      }
    }
  }

  c.terminate();

  std::ostringstream os;
  is_err >> os.rdbuf();

  // Remove new line from output
  std::string message = os.str();
  message.erase(std::remove(message.begin(), message.end(), '\n'), message.cend());
  syslog(LOG_INFO, "%s\n", message.c_str());

  result_.error_code = c.exit_code();
  result_.output = message;
}

std::string TrafficReaderService::get_command_line(const std::string & line)
{
  std::vector<std::string> tag_tokens;
  std::vector<std::string> slash_tokens;

  // usr/share/code/code/3102772/1000 0.565234 1.06855
  // Get first token separated by tab
  boost::split(tag_tokens, line, boost::is_any_of("\t"), boost::token_compress_on);

  // usr/share/code/code/3102772/1000
  // Get second from the last separated by slash
  boost::split(
    slash_tokens, tag_tokens[0].c_str(), boost::is_any_of("/"), boost::token_compress_on);
  if (slash_tokens.size() >= 3) {
    return get_command_line_with_pid(std::atoi(slash_tokens[slash_tokens.size() - 2].c_str()));
  }

  return "";
}

std::string TrafficReaderService::get_command_line_with_pid(pid_t pid)
{
  std::ifstream file(fmt::format("/proc/{}/cmdline", pid));

  if (!file) {
    return "";
  }

  std::string line;
  getline(file, line);

  // cmdline is null separated, replace with space
  std::replace(line.begin(), line.end(), '\0', ' ');

  return line;
}

}  // namespace traffic_reader_service
