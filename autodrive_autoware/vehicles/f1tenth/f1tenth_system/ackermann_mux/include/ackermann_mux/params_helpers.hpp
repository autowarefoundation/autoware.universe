// Copyright 2020 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * @author Paul Mathieu
 * @author Jeremie Deray
 * @author Hongrui Zheng
 */

#ifndef ACKERMANN_MUX__PARAMS_HELPERS_HPP_
#define ACKERMANN_MUX__PARAMS_HELPERS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <sstream>
#include <string>

namespace ackermann_mux
{
class ParamsHelperException : public std::runtime_error
{
public:
  explicit ParamsHelperException(const std::string & what)
  : std::runtime_error(what)
  {
  }
};

template<class T>
void fetch_param(std::shared_ptr<rclcpp::Node> nh, const std::string & param_name, T & output)
{
  rclcpp::Parameter param;
  if (!nh->get_parameter(param_name, param)) {
    std::ostringstream err_msg;
    err_msg << "could not load parameter '" << param_name << "'. (namespace: " <<
      nh->get_namespace() << ")";
    throw ParamsHelperException(err_msg.str());
  }

  output = param.get_value<T>();
}

}  // namespace ackermann_mux

#endif  // ACKERMANN_MUX__PARAMS_HELPERS_HPP_
