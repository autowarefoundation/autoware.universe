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
 * @author Enrique Fernandez
 * @author Jeremie Deray
 * @author Brighten Lee
 * @author Hongrui Zheng
 */

#ifndef ACKERMANN_MUX__ACKERMANN_MUX_DIAGNOSTICS_STATUS_HPP_
#define ACKERMANN_MUX__ACKERMANN_MUX_DIAGNOSTICS_STATUS_HPP_

#include <ackermann_mux/ackermann_mux.hpp>
#include <ackermann_mux/topic_handle.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace ackermann_mux
{
struct AckermannMuxDiagnosticsStatus
{
  typedef std::shared_ptr<AckermannMuxDiagnosticsStatus> Ptr;
  typedef std::shared_ptr<const AckermannMuxDiagnosticsStatus> ConstPtr;

  double reading_age;
  rclcpp::Time last_loop_update;
  double main_loop_time;

  LockTopicHandle::priority_type priority;

  std::shared_ptr<AckermannMux::velocity_topic_container> velocity_hs;
  std::shared_ptr<AckermannMux::lock_topic_container> lock_hs;

  AckermannMuxDiagnosticsStatus()
  : reading_age(0),
    last_loop_update(rclcpp::Clock().now()),
    main_loop_time(0),
    priority(0)
  {
    velocity_hs = std::make_shared<AckermannMux::velocity_topic_container>();
    lock_hs = std::make_shared<AckermannMux::lock_topic_container>();
  }
};

typedef AckermannMuxDiagnosticsStatus::Ptr AckermannMuxDiagnosticsStatusPtr;
typedef AckermannMuxDiagnosticsStatus::ConstPtr AckermannMuxDiagnosticsStatusConstPtr;

}  // namespace ackermann_mux

#endif  // ACKERMANN_MUX__ACKERMANN_MUX_DIAGNOSTICS_STATUS_HPP_
