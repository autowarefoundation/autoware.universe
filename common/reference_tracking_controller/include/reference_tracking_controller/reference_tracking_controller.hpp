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
/// \brief This class defines an interface for simple reference tracking controllers
#ifndef REFERENCE_TRACKING_CONTROLLER__REFERENCE_TRACKING_CONTROLLER_HPP_
#define REFERENCE_TRACKING_CONTROLLER__REFERENCE_TRACKING_CONTROLLER_HPP_

#include <reference_tracking_controller/visibility_control.hpp>

#include <type_traits>

namespace autoware
{
namespace common
{
namespace reference_tracking_controller
{
/// Interface class for 1D reference trackers
/// \tparam T A floating point type
template<typename T>
class REFERENCE_TRACKING_CONTROLLER_PUBLIC ReferenceTrackerBase
{
  static_assert(
    std::is_floating_point<T>::value,
    "Reference trackers only work for floating point types");

public:
  /// A Pack of both the signal and it's derivative at some time step
  struct SignalWithDerivative
  {
    T value;  ///< Value of signal at current time step
    T derivative;  ///< Derivative or rate of signal at current time step
  };  // struct SignalWithDerivative
  /// Virtual destructor
  virtual ~ReferenceTrackerBase() = default;
  /// Primary API: given current reference and current observation (and derivatives),
  /// compute feedback control
  /// \param[in] reference The value being tracked, sometimes referred to as y
  /// \param[in] observation The current observed value of the tracked value, sometimes referred to
  ///                        as z or o
  /// \return The feedback value
  virtual T feedback(SignalWithDerivative reference, SignalWithDerivative observation) = 0;

  /// Primary API: given current reference and current observation (and derivatives),
  /// compute feedback control. Reference derivative is assumed to be 0
  /// \param[in] reference The value being tracked, sometimes referred to as y
  /// \param[in] observation The current observed value of the tracked value, sometimes referred to
  ///                        as z or o, with derivative
  /// \return The feedback value
  T feedback(T reference, SignalWithDerivative observation)
  {
    return feedback({reference, T{}}, observation);
  }

  /// Primary API: given current reference and current observation (and derivatives),
  /// compute feedback control. Observation derivative is assumed to be zero
  /// \param[in] reference The value being tracked, sometimes referred to as y, with derivative
  /// \param[in] observation The current observed value of the tracked value, sometimes referred to
  ///                        as z or o
  /// \return The feedback value
  T feedback(SignalWithDerivative reference, T observation)
  {
    return feedback(reference, {observation, T{}});
  }

  /// Primary API: given current reference and current observation,
  /// compute feedback control. All derivatives are assumed to be zero
  /// \param[in] reference The value being tracked, sometimes referred to as y
  /// \param[in] observation The current observed value of the tracked value, sometimes referred to
  ///                        as z or o
  /// \return The feedback value
  T feedback(T reference, T observation)
  {
    return feedback({reference, T{}}, {observation, T{}});
  }
};  // ReferenceTrackerBase
}  // namespace reference_tracking_controller
}  // namespace common
}  // namespace autoware

#endif  // REFERENCE_TRACKING_CONTROLLER__REFERENCE_TRACKING_CONTROLLER_HPP_
