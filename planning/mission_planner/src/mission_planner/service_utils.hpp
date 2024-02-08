// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef MISSION_PLANNER__SERVICE_UTILS_HPP_
#define MISSION_PLANNER__SERVICE_UTILS_HPP_

#include <autoware_common_msgs/msg/response_status.hpp>

#include <stdexcept>
#include <string>

namespace service_utils
{

using ResponseStatus = autoware_common_msgs::msg::ResponseStatus;
using ResponseStatusCode = ResponseStatus::_code_type;

class ServiceException : public std::exception
{
public:
  ServiceException(ResponseStatusCode code, const std::string & message, bool success = false)
  {
    success_ = success;
    code_ = code;
    message_ = message;
  }

  template <class StatusT>
  void set(StatusT & status) const
  {
    status.success = success_;
    status.code = code_;
    status.message = message_;
  }

private:
  bool success_;
  ResponseStatusCode code_;
  std::string message_;
};

ServiceException ServiceUnready(const std::string & message);
ServiceException TransformError(const std::string & message);

template <class T, class Req, class Res>
std::function<void(Req, Res)> handle_exception(void (T::*callback)(Req, Res), T * instance)
{
  return [instance, callback](Req req, Res res) {
    try {
      (instance->*callback)(req, res);
    } catch (const ServiceException & error) {
      error.set(res->status);
    }
  };
}

template <class T, class Req>
ResponseStatus sync_call(T & client, Req req)
{
  if (!client->service_is_ready()) {
    throw ServiceUnready(client->get_service_name());
  }
  auto future = client->async_send_request(req);
  return future.get()->status;
}

}  // namespace service_utils

#endif  // MISSION_PLANNER__SERVICE_UTILS_HPP_
