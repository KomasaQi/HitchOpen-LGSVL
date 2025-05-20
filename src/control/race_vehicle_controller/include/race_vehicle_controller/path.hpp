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
 * @file path.hpp
 * @author Haoru Xue (haorux@andrew.cmu.edu)
 * @brief path pointers
 * @version 0.1
 * @date 2022-01-01
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#ifndef RACE_VEHICLE_CONTROLLER__PATH_HPP_
#define RACE_VEHICLE_CONTROLLER__PATH_HPP_

#include <vector>
#include <memory>
#include "ttl.hpp"

using race::ttl::Waypoint;

namespace race
{
typedef std::vector<Waypoint> Path;
typedef std::shared_ptr<Path> PathSharedPtr;
}

#endif  // RACE_VEHICLE_CONTROLLER__PATH_HPP_
