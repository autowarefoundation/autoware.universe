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

#ifndef ACKERMANN_MUX__ACKERMANN_MUX_DIAGNOSTICS_HPP_
#define ACKERMANN_MUX__ACKERMANN_MUX_DIAGNOSTICS_HPP_

#include <ackermann_mux/ackermann_mux_diagnostics_status.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <memory>

namespace ackermann_mux
{
class AckermannMuxDiagnostics
{
public:
  typedef AckermannMuxDiagnosticsStatus status_type;

  static constexpr double MAIN_LOOP_TIME_MIN = 0.2;   // [s]
  static constexpr double READING_AGE_MIN = 3.0;     // [s]

  explicit AckermannMuxDiagnostics(AckermannMux * mux);
  virtual ~AckermannMuxDiagnostics() = default;

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void update();

  void updateStatus(const status_type::ConstPtr & status);

private:
  /**
   * @brief Levels
   */
  enum
  {
    OK = diagnostic_msgs::msg::DiagnosticStatus::OK,
    WARN = diagnostic_msgs::msg::DiagnosticStatus::WARN,
    ERROR = diagnostic_msgs::msg::DiagnosticStatus::ERROR
  };

  std::shared_ptr<diagnostic_updater::Updater> diagnostic_;
  std::shared_ptr<status_type> status_;
};
}  // namespace ackermann_mux

#endif  // ACKERMANN_MUX__ACKERMANN_MUX_DIAGNOSTICS_HPP_
