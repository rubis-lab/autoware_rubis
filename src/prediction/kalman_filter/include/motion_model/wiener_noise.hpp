// Copyright 2021 the Autoware Foundation
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
// Developed by Apex.AI, Inc.

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief This file contains the Wiener noise model classes.

#ifndef MOTION_MODEL__WIENER_NOISE_HPP_
#define MOTION_MODEL__WIENER_NOISE_HPP_

#include <kalman_filter/common_states.hpp>
#include <kalman_filter/visibility_control.hpp>
#include <motion_model/noise_interface.hpp>

#include <array>

namespace autoware
{
namespace prediction
{

///
/// @brief      A class that describes the Wiener process noise.
///
///             For more details see notebook here:
///             https://nbviewer.jupyter.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/
///             blob/master/07-Kalman-Filter-Math.ipynb#Piecewise-White-Noise-Model (combine into
///             one line)
///
/// @tparam     StateT  A given state type.
///
template<typename StateT>
class KALMAN_FILTER_PUBLIC WienerNoise : public NoiseInterface<WienerNoise<StateT>>
{
public:
  using State = StateT;

  ///
  /// @brief      Constructor from acceleration variances.
  ///
  /// @param[in]  acceleration_variances  The acceleration variances, note that these are sigmas,
  ///                                     not sigmas squared.
  ///
  explicit WienerNoise(
    const std::array<typename State::Scalar, State::size()> & acceleration_variances)
  : m_acceleration_variances{acceleration_variances} {}

protected:
  // Required to allow the crtp interface call the following functions.
  friend NoiseInterface<WienerNoise<StateT>>;

  ///
  /// @brief      A CRTP-called covariance getter.
  ///
  /// @return     A covariance of the noise process over given time.
  ///
  typename State::Matrix crtp_covariance(const std::chrono::nanoseconds &) const
  {
    static_assert(
      sizeof(StateT) == 0U,
      "\n\nThis function must be specialized for specific states.\n\n");
  }

private:
  std::array<typename State::Scalar, State::size()> m_acceleration_variances{};
};

///
/// @brief      A specialization of covariance matrix computation for ConstAccelerationXY state.
///
/// @param[in]  dt    Time step.
///
/// @return     Covariance matrix.
///
template<>
KALMAN_FILTER_PUBLIC state::ConstAccelerationXY::Matrix
WienerNoise<state::ConstAccelerationXY>::crtp_covariance(
  const std::chrono::nanoseconds & dt) const;

///
/// @brief      A specialization of covariance matrix computation for ConstAccelerationXYYaw state.
///
/// @param[in]  dt    Time step.
///
/// @return     Covariance matrix.
///
template<>
KALMAN_FILTER_PUBLIC state::ConstAccelerationXYYaw::Matrix
WienerNoise<state::ConstAccelerationXYYaw>::crtp_covariance(
  const std::chrono::nanoseconds & dt) const;

}  // namespace prediction
}  // namespace autoware

#endif  // MOTION_MODEL__WIENER_NOISE_HPP_
