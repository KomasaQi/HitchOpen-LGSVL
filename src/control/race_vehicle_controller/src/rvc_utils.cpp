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
 * @file rvc_utils.cpp
 * @author Haoru Xue (haorux@andrew.cmu.edu)
 * @brief rvc utils
 * @version 0.1
 * @date 2022-08-04
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#include <vector>
#include "race_vehicle_controller/rvc_utils.hpp"

namespace race
{
namespace utils
{
double interpolate(
  const std::vector<double> & xData, const std::vector<double> & yData,
  const double & x, const bool & extrapolate)
{
  int size = xData.size();
  int i = 0;
  if (x >= xData[size - 2]) {
    i = size - 2;
  } else {
    while (x > xData[i + 1]) {i++;}
  }
  double xL = xData[i], yL = yData[i], xR = xData[i + 1], yR = yData[i + 1];
  if (!extrapolate) {
    if (x < xL) {yR = yL;}
    if (x > xR) {yL = yR;}
  }
  double dydx = ( yR - yL ) / ( xR - xL );
  return yL + dydx * ( x - xL );
}
}  // namespace utils
}  // namespace race
