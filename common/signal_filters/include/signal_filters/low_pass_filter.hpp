// Copyright 2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief Interface definition for signal processing filters
#ifndef SIGNAL_FILTERS__LOW_PASS_FILTER_HPP_
#define SIGNAL_FILTERS__LOW_PASS_FILTER_HPP_

#include <signal_filters/signal_filter.hpp>
#include <signal_filters/visibility_control.hpp>

#include <chrono>

namespace autoware
{
namespace common
{
namespace signal_filters
{

/// Basic low pass filter implemented as an exponential moving average
/// \tparam T A floating point type for the signal
template<typename T, typename ClockT = std::chrono::steady_clock>
class SIGNAL_FILTERS_PUBLIC LowPassFilter : public FilterBase<T, ClockT>
{
public:
  explicit LowPassFilter(T cutoff_frequency_hz)
  : FilterBase<T, ClockT>{}
  {
    if (T{} >= cutoff_frequency_hz) {
      throw std::domain_error{"Cutoff frequency is non-positve"};
    }
    constexpr T TAU{static_cast<T>(2.0 * 3.14159)};
    m_rc_inv = TAU * cutoff_frequency_hz;
  }
  /// Destructor
  virtual ~LowPassFilter() = default;

protected:
  T filter_impl(T value, std::chrono::nanoseconds duration) override
  {
    const auto dt = std::chrono::duration_cast<std::chrono::duration<T>>(duration).count();
    // From https://stackoverflow.com/a/1027808
    const auto alpha = T{1.0} - std::exp(-dt * m_rc_inv);
    m_signal += alpha * (value - m_signal);
    return m_signal;
  }

private:
  T m_rc_inv{};
  T m_signal{};
};
}  // namespace signal_filters
}  // namespace common
}  // namespace autoware

#endif  // SIGNAL_FILTERS__LOW_PASS_FILTER_HPP_
