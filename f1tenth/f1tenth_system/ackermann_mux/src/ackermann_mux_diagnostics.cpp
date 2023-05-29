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
 * @author Brighten Lee
 * @author Hongrui Zheng
 */

#include <ackermann_mux/ackermann_mux_diagnostics.hpp>
#include <ackermann_mux/ackermann_mux_diagnostics_status.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <memory>

namespace ackermann_mux
{
AckermannMuxDiagnostics::AckermannMuxDiagnostics(AckermannMux * mux)
{
  diagnostic_ = std::make_shared<diagnostic_updater::Updater>(mux);
  status_ = std::make_shared<status_type>();

  diagnostic_->add("Ackermann mux status", this, &AckermannMuxDiagnostics::diagnostics);
  diagnostic_->setHardwareID("none");
}

void AckermannMuxDiagnostics::update()
{
  diagnostic_->force_update();
}

void AckermannMuxDiagnostics::updateStatus(const status_type::ConstPtr & status)
{
  status_->velocity_hs = status->velocity_hs;
  status_->lock_hs = status->lock_hs;
  status_->priority = status->priority;

  status_->main_loop_time = status->main_loop_time;
  status_->reading_age = status->reading_age;

  update();
}

void AckermannMuxDiagnostics::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  /// Check if the loop period is quick enough
  if (status_->main_loop_time > MAIN_LOOP_TIME_MIN) {
    stat.summary(ERROR, "loop time too long");
  } else if (status_->reading_age > READING_AGE_MIN) {
    stat.summary(ERROR, "data received is too old");
  } else {
    stat.summary(OK, "ok");
  }

  for (auto & velocity_h : *status_->velocity_hs) {
    stat.addf(
      "velocity " + velocity_h.getName(), " %s (listening to %s @ %fs with priority #%d)",
      (velocity_h.isMasked(status_->priority) ? "masked" : "unmasked"),
      velocity_h.getTopic().c_str(),
      velocity_h.getTimeout().seconds(), static_cast<int>(velocity_h.getPriority()));
  }

  for (const auto & lock_h : *status_->lock_hs) {
    stat.addf(
      "lock " + lock_h.getName(), " %s (listening to %s @ %fs with priority #%d)",
      (lock_h.isLocked() ? "locked" : "free"), lock_h.getTopic().c_str(),
      lock_h.getTimeout().seconds(),
      static_cast<int>(lock_h.getPriority()));
  }

  stat.add("current priority", static_cast<int>(status_->priority));

  stat.add("loop time in [sec]", status_->main_loop_time);
  stat.add("data age in [sec]", status_->reading_age);
}

}  // namespace ackermann_mux
