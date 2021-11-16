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
#ifndef SIGNAL_FILTERS__FILTER_FACTORY_HPP_
#define SIGNAL_FILTERS__FILTER_FACTORY_HPP_

#include <signal_filters/visibility_control.hpp>
#include <signal_filters/signal_filter.hpp>
#include <signal_filters/low_pass_filter.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

namespace autoware
{
namespace common
{
namespace signal_filters
{

enum class FilterType : int32_t
{
  None = 0,
  LowPassFilter
};  // enum class FilterType

/// Factory class to create some kind of low pass filter
class SIGNAL_FILTERS_PUBLIC FilterFactory
{
public:
  /// Create a low pass filter
  /// \param[in] type The name of the filter type, expects one of "none, "low_pass_filter"
  /// \param[in] cutoff_frequency The filter starts to have < 1.0 frequency response starting
  ///                             around here
  /// \tparam T The floating point type of the filter
  /// \tparam ClockT The clock type of the filter. Use default parameter if you want to provide
  ///                durations between points yourself
  template<typename T, typename ClockT = DummyClock>
  static std::unique_ptr<FilterBase<T, ClockT>> create(const std::string & type, T cutoff_frequency)
  {
    FilterType type_enum = FilterType::LowPassFilter;
    auto type_clean = type;
    (void)std::transform(
      type_clean.begin(), type_clean.end(), type_clean.begin(),
      [](auto c) {return std::tolower(c);});
    if ("low_pass_filter" == type_clean) {
      type_enum = FilterType::LowPassFilter;
    } else if (type_clean.empty() || ("none" == type_clean)) {
      type_enum = FilterType::None;
    } else {
      std::string err{"Unknown filter type: "};
      err += type;
      throw std::domain_error{err};
    }
    return create<T, ClockT>(type_enum, cutoff_frequency);
  }

  /// Create a low pass filter
  /// \param[in] type The type of the filter type
  /// \param[in] cutoff_frequency The filter starts to have < 1.0 frequency response starting
  ///                             around here
  /// \tparam T The floating point type of the filter
  /// \tparam ClockT The clock type of the filter. Use default parameter if you want to provide
  ///                durations between points yourself
  template<typename T, typename ClockT = DummyClock>
  static std::unique_ptr<FilterBase<T, ClockT>> create(FilterType type, T cutoff_frequency)
  {
    switch (type) {
      case FilterType::None:
        return nullptr;
      case FilterType::LowPassFilter:
        return std::make_unique<LowPassFilter<T, ClockT>>(cutoff_frequency);
      default:
        throw std::domain_error{"Unknown filter type"};
    }
  }
};  // class FilterFactory

}  // namespace signal_filters
}  // namespace common
}  // namespace autoware

#endif  // SIGNAL_FILTERS__FILTER_FACTORY_HPP_
