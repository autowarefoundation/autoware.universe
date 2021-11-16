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
#include <common/types.hpp>
#include <gtest/gtest.h>

#include <signal_filters/filter_factory.hpp>
#include <helper_functions/float_comparisons.hpp>

#include <chrono>
#include <cmath>
#include <complex>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace comp = autoware::common::helper_functions::comparisons;

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::signal_filters::FilterFactory;
using autoware::common::signal_filters::DummyClock;
using autoware::common::signal_filters::FilterType;

template<typename FloatT, typename ClockT>
struct TypeParams
{
  using Float = FloatT;
  using Clock = ClockT;
};

template<typename T>
class TypedChecks : public ::testing::Test
{
};

using TestTypes = ::testing::Types<
  TypeParams<float32_t, DummyClock>,
  TypeParams<float32_t, std::chrono::system_clock>,
  TypeParams<float32_t, std::chrono::steady_clock>,
  TypeParams<float64_t, DummyClock>,
  TypeParams<float64_t, std::chrono::system_clock>,
  TypeParams<float64_t, std::chrono::steady_clock>
>;
// cppcheck-suppress syntaxError
TYPED_TEST_SUITE(TypedChecks, TestTypes, );

TYPED_TEST(TypedChecks, EmptyFactory)
{
  using Clock = typename TypeParam::Clock;
  using Float = typename TypeParam::Float;
  EXPECT_EQ(nullptr, (FilterFactory::create<Float, Clock>("", 1.0F)));
  EXPECT_EQ(nullptr, (FilterFactory::create<Float, Clock>("none", 1.0F)));
  EXPECT_THROW((FilterFactory::create<Float, Clock>("foo", 1.0F)), std::domain_error);
}

///////////////////////////////////////////////////////////////////////////////
template<typename FloatT, typename ClockT, FilterType TypeS>
struct BasicParams
{
  using Float = FloatT;
  using Clock = ClockT;
  static constexpr FilterType Type = TypeS;
};

template<typename T>
class FilterChecks : public ::testing::Test
{
protected:
  template<typename ClockT, typename = std::enable_if_t<!std::is_same<ClockT, DummyClock>::value>>
  typename ClockT::time_point one()
  {
    return typename ClockT::time_point{} + std::chrono::nanoseconds{1LL};
  }
  template<typename ClockT, typename = std::enable_if_t<std::is_same<ClockT, DummyClock>::value>>
  std::chrono::nanoseconds one()
  {
    return std::chrono::nanoseconds{1LL};
  }
};

using BasicTypes = ::testing::Types<
  BasicParams<float32_t, DummyClock, FilterType::LowPassFilter>,
  BasicParams<float32_t, std::chrono::system_clock, FilterType::LowPassFilter>,
  BasicParams<float32_t, std::chrono::steady_clock, FilterType::LowPassFilter>,
  BasicParams<float64_t, DummyClock, FilterType::LowPassFilter>,
  BasicParams<float64_t, std::chrono::system_clock, FilterType::LowPassFilter>,
  BasicParams<float64_t, std::chrono::steady_clock, FilterType::LowPassFilter>
>;
TYPED_TEST_SUITE(FilterChecks, BasicTypes, );

TYPED_TEST(FilterChecks, BadFactory)
{
  using Clock = typename TypeParam::Clock;
  using Float = typename TypeParam::Float;
  EXPECT_THROW((FilterFactory::create<Float, Clock>(TypeParam::Type, -1.0F)), std::domain_error);
}

TYPED_TEST(FilterChecks, BadInput)
{
  using Clock = typename TypeParam::Clock;
  using Float = typename TypeParam::Float;
  const auto filter = FilterFactory::create<Float, Clock>(TypeParam::Type, 1.0F);
  // Bad value
  const auto t = this->template one<Clock>();
  constexpr auto nan1 = std::numeric_limits<Float>::quiet_NaN();
  constexpr auto nan2 = std::numeric_limits<Float>::signaling_NaN();
  constexpr auto inf = std::numeric_limits<Float>::infinity();
  EXPECT_THROW(filter->filter(nan1, t), std::domain_error);
  EXPECT_THROW(filter->filter(nan2, t), std::domain_error);
  EXPECT_THROW(filter->filter(inf, t), std::domain_error);
  EXPECT_THROW(filter->filter(-nan1, t), std::domain_error);
  EXPECT_THROW(filter->filter(-nan2, t), std::domain_error);
  EXPECT_THROW(filter->filter(-inf, t), std::domain_error);
  // Bad duration
  EXPECT_NO_THROW(filter->filter(Float{}, t));
  const auto bad_t = t - std::chrono::nanoseconds{2LL};  // Backwards in time or negative duration
  EXPECT_THROW(filter->filter(Float{}, bad_t), std::domain_error);
}

///////////////////////////////////////////////////////////////////////////////
template<typename Real, typename Clock>
class Helper
{
public:
  template<typename RealT, typename ClockT>
  using FilterBase = autoware::common::signal_filters::FilterBase<RealT, ClockT>;

  struct SignalPoint
  {
    Real value;
    typename Clock::time_point time;
  };

  using Signal = std::vector<SignalPoint>;
  // Key = Frequency, Value = Magnitude of frequency response
  using FrequencyResponse = std::map<Real, Real>;

  /// Simple N^2 DFT implementation that only gives the magnitude of the frequency response
  static FrequencyResponse dft(const Signal & signal)
  {
    // https://en.wikipedia.org/wiki/Non-uniform_discrete_Fourier_transform
    FrequencyResponse frequency_response{};

    const auto period = std::chrono::duration_cast<std::chrono::duration<Real>>(
      signal.back().time - signal.front().time).count();
    constexpr auto TAU = 2.0 * 3.14159;
    using T = typename FrequencyResponse::mapped_type;
    const auto freq_base = std::complex<T>{0.0, -TAU / static_cast<T>(signal.size())};
    for (auto idx = 0U; idx < signal.size(); ++idx) {
      const auto freq_component = freq_base * static_cast<T>(idx);
      std::complex<T> response{};
      for (auto jdx = 0U; jdx < signal.size(); ++jdx) {
        const auto freq = freq_component * static_cast<T>(jdx);
        response += signal[jdx].value * std::exp(freq);
      }
      frequency_response[static_cast<Real>(idx) / period] = std::abs(response);
    }

    return frequency_response;
  }

  // Create a basic binary signal
  static Signal signal(Real frequency, Real magnitude, std::size_t count)
  {
    Signal ret{};
    ret.reserve(count);
    // Compute period
    const auto period = std::chrono::duration<Real>(1.0 / frequency);
    const auto dt_2 = std::chrono::duration_cast<std::chrono::nanoseconds>(period) / 2;
    // Start
    const auto start = Clock::now();
    for (auto idx = 0U; idx < count; ++idx) {
      const auto val = (idx % 2U == 0U) ? magnitude : -magnitude;
      ret.emplace_back(SignalPoint{val, start + (idx * dt_2)});
    }
    return ret;
  }

  // Check if a signal has a frequency response beyond some threshold
  static bool8_t has_response_past_cutoff(
    Real cutoff_frequency,
    Real raw_signal_magnitude,
    const FrequencyResponse & response)
  {
    // Iterate over frequency response to ensure that nothing exceeds specified magnitude
    for (const auto & resp : response) {
      const auto frequency = resp.first;
      const auto magnitude = resp.second;
      if ((frequency >= cutoff_frequency) && (magnitude >= raw_signal_magnitude)) {
        return true;
      }
    }
    return false;
  }

  // Apply filter
  template<typename DummyT = Clock>
  static std::enable_if_t<!std::is_same<DummyT, DummyClock>::value, Signal>
  filter_signal(FilterBase<Real, DummyT> & filter, const Signal & signal)
  {
    Signal ret{};
    ret.reserve(signal.size());
    for (const auto & pt : signal) {
      const auto val = filter.filter(pt.value, pt.time);
      ret.emplace_back(SignalPoint{val, pt.time});
    }
    return ret;
  }
  template<typename DummyT = Clock>
  static std::enable_if_t<std::is_same<DummyT, DummyClock>::value, Signal>
  filter_signal(FilterBase<Real, DummyT> & filter, const Signal & signal)
  {
    Signal ret{};
    ret.reserve(signal.size());
    for (auto idx = 0U; idx < signal.size(); ++idx) {
      const auto & pt = signal[idx];
      const auto dt = (idx != 0U) ? pt.time - signal[idx - 1U].time : std::chrono::nanoseconds{1LL};
      const auto val = filter.filter(pt.value, dt);
      ret.emplace_back(SignalPoint{val, pt.time});
    }
    return ret;
  }
};

template<typename FloatT, typename ClockT, FilterType TypeE,
  int SignalMagnitude, int SignalFrequency, int FrequencyCutoff>
struct SanityCheckParam
{
  using Float = FloatT;
  using Clock = ClockT;
  static constexpr FilterType filter_type = TypeE;
  static constexpr Float cutoff_frequency = static_cast<Float>(FrequencyCutoff);
  static constexpr Float signal_frequency = static_cast<Float>(SignalFrequency);
  static constexpr Float signal_magnitude = static_cast<Float>(SignalMagnitude);
};

template<typename T>
class SanityCheck : public ::testing::Test
{
};

using SanityCheckTypes = ::testing::Types<
  SanityCheckParam<double, DummyClock, FilterType::LowPassFilter, 1, 3, 1>,
  SanityCheckParam<double, std::chrono::system_clock, FilterType::LowPassFilter, 1, 3, 1>,
  SanityCheckParam<double, std::chrono::steady_clock, FilterType::LowPassFilter, 1, 3, 1>,
  SanityCheckParam<double, DummyClock, FilterType::LowPassFilter, 1, 100, 1>,
  SanityCheckParam<double, std::chrono::system_clock, FilterType::LowPassFilter, 1, 100, 1>,
  SanityCheckParam<double, std::chrono::steady_clock, FilterType::LowPassFilter, 1, 100, 1>,
  SanityCheckParam<double, DummyClock, FilterType::LowPassFilter, 1, 3, 1>,
  SanityCheckParam<double, std::chrono::system_clock, FilterType::LowPassFilter, 1, 3, 1>,
  SanityCheckParam<double, std::chrono::steady_clock, FilterType::LowPassFilter, 1, 3, 1>,
  SanityCheckParam<double, DummyClock, FilterType::LowPassFilter, 1, 100, 1>,
  SanityCheckParam<double, std::chrono::system_clock, FilterType::LowPassFilter, 1, 100, 1>,
  SanityCheckParam<double, std::chrono::steady_clock, FilterType::LowPassFilter, 1, 100, 1>
>;
TYPED_TEST_SUITE(SanityCheck, SanityCheckTypes, );

TYPED_TEST(SanityCheck, Basic)
{
  using Real = typename TypeParam::Float;
  using Clock = typename TypeParam::Clock;
  using HelperT = Helper<Real, Clock>;
  constexpr auto count = 100U;
  const auto filter =
    FilterFactory::create<Real, Clock>(TypeParam::filter_type, TypeParam::cutoff_frequency);
  const auto raw_signal =
    HelperT::signal(TypeParam::signal_frequency, TypeParam::signal_magnitude, count);
  // Compute FFT of raw signal
  const auto raw_response = HelperT::dft(raw_signal);
  ASSERT_TRUE(
    HelperT::has_response_past_cutoff(
      TypeParam::cutoff_frequency,
      TypeParam::signal_magnitude,
      raw_response));
  // And find maximum frequency response
  const auto response_magnitude = std::max_element(
    raw_response.begin(), raw_response.end(),
    [](auto a, auto b) {return a.second < b.second;})->second;
  // Pass through filter
  const auto filtered_signal = HelperT::filter_signal(*filter, raw_signal);
  // Compute FFT of filtered signal
  const auto filtered_response = HelperT::dft(filtered_signal);
  EXPECT_FALSE(
    HelperT::has_response_past_cutoff(
      TypeParam::cutoff_frequency,
      response_magnitude,
      filtered_response));
  // TODO(c.ho) Someone with good signal processing should figure out a better way to check response
  // matches theoretical performance of filter
  auto has_nonzero_response = false;
  for (const auto & resp : filtered_response) {
    constexpr auto EPS = std::numeric_limits<decltype(resp.second)>::epsilon();
    if (!comp::abs_eq_zero(resp.second, EPS)) {
      has_nonzero_response = true;
    }
  }
  EXPECT_TRUE(has_nonzero_response);
}
