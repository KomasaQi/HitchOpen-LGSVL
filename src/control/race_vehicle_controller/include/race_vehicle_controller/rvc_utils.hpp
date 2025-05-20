// Copyright 2022 AI Racing Tech
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


/**
 * @file rvc_utils.hpp
 * @author Haoru Xue (haorux@andrew.cmu.edu)
 * @brief rvc utils
 * @version 0.1
 * @date 2022-08-04
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#ifndef RACE_VEHICLE_CONTROLLER__RVC_UTILS_HPP_
#define RACE_VEHICLE_CONTROLLER__RVC_UTILS_HPP_

#include <vector>

namespace race
{
namespace utils
{
double interpolate(
  const std::vector<double> & xData, const std::vector<double> & yData,
  const double & x, const bool & extrapolate);
}  // namespace utils
}  // namespace race

#endif  // RACE_VEHICLE_CONTROLLER__RVC_UTILS_HPP_
