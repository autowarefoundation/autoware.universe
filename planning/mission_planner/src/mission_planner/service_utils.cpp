// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#include "service_utils.hpp"

#include <string>

namespace service_utils
{

ServiceException ServiceUnready(const std::string & message)
{
  return ServiceException(ResponseStatus::SERVICE_UNREADY, message, false);
}

ServiceException TransformError(const std::string & message)
{
  return ServiceException(ResponseStatus::TRANSFORM_ERROR, message, false);
};

}  // namespace service_utils
