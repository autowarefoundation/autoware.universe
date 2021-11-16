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
#ifndef SIGNAL_FILTERS__SIGNAL_FILTER_HPP_
#define SIGNAL_FILTERS__SIGNAL_FILTER_HPP_

#include <common/types.hpp>
#include <signal_filters/visibility_control.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <type_traits>

using autoware::common::types::bool8_t;

namespace autoware
{
namespace common
{
namespace signal_filters
{

/// Fake clock to denote that the duration API is supposed to be used
struct DummyClock
{
  static auto now() {return std::chrono::steady_clock::now();}  // Just for test convenience
  using time_point = std::chrono::steady_clock::time_point;  // Just for test convenience
};

/// Interface class for filters in the signal processing sense; attenuate response from
/// various frequency domains
/// \tparam T A floating point type for the signal
/// \tparam ClockT The (std::)clock type intended to be used. Setting this implies that the
///               time_point-based API is going to be used (and that the duration API can't be used)
template<typename T, typename ClockT = DummyClock>
class SIGNAL_FILTERS_PUBLIC FilterBase
{
  static_assert(std::is_floating_point<T>::value, "Filters require a floating point type");
  constexpr static bool8_t use_time_point_api = !std::is_same<ClockT, DummyClock>::value;

public:
  using clock_type = ClockT;
  using signal_type = T;

  /// Destructor
  virtual ~FilterBase() = default;
  /// Primary API: receives a value and outputs the result of the filter
  /// \param[in] value An observation on the signal
  /// \param[in] time_stamp The time of the current observation
  /// \return The result of the filter
  /// \throw std::domain_error If time_stamp goes back in time
  /// \throw std::domain_error If value is not normal
  /// \tparam DummyT Dummy type to get SFINAE to work
  template<typename DummyT = T, typename = std::enable_if_t<use_time_point_api, DummyT>>
  T filter(T value, typename clock_type::time_point time_stamp)
  {
    const auto dt = time_stamp - m_last_observation_stamp;
    const auto ret = filter_impl_checked(value, dt);
    m_last_observation_stamp = time_stamp;  // Goes after just in case there's an exception
    return ret;
  }
  /// Primary API: receives a value and outputs the result of the filter
  /// \param[in] value An observation on the signal
  /// \param[in] duration Time since last observation, must be positive
  /// \return The result of the filter
  /// \throw std::domain_error If duration is negative
  /// \throw std::domain_error If value is not normal
  /// \tparam DummyT Dummy type to get SFINAE to work
  template<typename DummyT = T, typename = std::enable_if_t<!use_time_point_api, DummyT>>
  T filter(T value, std::chrono::nanoseconds duration)
  {
    return filter_impl_checked(value, duration);
  }

protected:
  /// Underlying implementation, with error checking
  T filter_impl_checked(T value, std::chrono::nanoseconds duration)
  {
    if (decltype(duration)::zero() >= duration) {
      throw std::domain_error{"Duration is negative"};
    }
    if (!std::isfinite(value)) {
      throw std::domain_error{"Value is not finite"};
    }
    return filter_impl(value, duration);
  }
  /// Actual implementation, error checking already done, can assume duration is positive
  virtual T filter_impl(T value, std::chrono::nanoseconds duration) = 0;

private:
  typename clock_type::time_point m_last_observation_stamp{};
};

}  // namespace signal_filters
}  // namespace common
}  // namespace autoware

#endif  // SIGNAL_FILTERS__SIGNAL_FILTER_HPP_
