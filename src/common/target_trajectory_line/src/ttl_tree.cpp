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

#include <vector>
#include <memory>
#include <unordered_set>
#include <limits>

#include "ttl_tree.hpp"

#include "CGAL/Line_2.h"
#include "CGAL/Point_2.h"
#include "CGAL/Cartesian.h"
#include "CGAL/squared_distance_2.h"

namespace race::ttl
{
TtlTree::TtlTree() {}
TtlTree::TtlTree(const char * ttl_dir)
{
  initialize(ttl_dir);
}

void TtlTree::initialize(const char * ttl_dir)
{
  std::vector<uint8_t> valid_ttl_idx;
  load_all(ttl_dir, ttl_array_, valid_ttl_idx);
  std::cout << "Parsing TTL trees and dictionaries" << std::endl;
  // Initialize TTL maps and trees
  for (const auto & ttl_idx : valid_ttl_idx) {
    valid_ttl_idx_set_.emplace(ttl_idx);
    auto & map = maps_.maps.at(ttl_idx);
    auto & ttl = ttl_array_.trajectories.at(ttl_idx);
    auto & tree = trees_.trees.at(ttl_idx);
    map.reserve(ttl.header.loop);
    tree.reserve(ttl.header.loop);
    for (size_t i = 0; i < ttl.header.loop; i++) {
      const auto & wp = ttl.waypoints.at(i);
      map[WaypointKey{wp.location.x, wp.location.y}] = i;
      tree.insert(Point_d(wp.location.x, wp.location.y));
    }
    tree.build();
  }
  std::cout << "Finished parsing TTL trees and dictionaries" << std::endl;
}

size_t TtlTree::find_closest_waypoint_index(const TtlIndex & ttl_idx, const Position & ego) const
{
  const size_t ttl_num = static_cast<size_t>(ttl_idx);
  auto & map = maps_.maps.at(ttl_num);
  auto & tree = trees_.trees.at(ttl_num);
  Point_d query(ego.x, ego.y);
  Neighbor_search search(tree, query, 1);
  const auto & closest_pt = *search.begin();
  return map.find(WaypointKey{closest_pt.first.x(), closest_pt.first.y()})->second;
}

void TtlTree::find_closest_waypoint_indices(
  const TtlIndex & ttl_idx, const Position & ego,
  const size_t & n, std::vector<size_t> & indices) const
{
  const size_t ttl_num = static_cast<size_t>(ttl_idx);
  auto & map = maps_.maps.at(ttl_num);
  auto & tree = trees_.trees.at(ttl_num);
  Point_d query(ego.x, ego.y);
  Neighbor_search search(tree, query, n);
  indices.reserve(search.end() - search.begin());
  for (const auto & pt : search) {
    indices.push_back(map.find(WaypointKey{pt.first.x(), pt.first.y()})->second);
  }
}

const Ttl & TtlTree::get_ttl(const TtlIndex & ttl_index) const
{
  return ttl_array_.trajectories[static_cast<size_t>(ttl_index)];
}

const std::unordered_set<uint8_t> & TtlTree::get_valid_ttl_index_set() const
{
  return valid_ttl_idx_set_;
}

bool TtlTree::is_ttl_index_valid(const uint8_t & ttl_index) const
{
  return valid_ttl_idx_set_.find(ttl_index) != valid_ttl_idx_set_.end();
}


bool TtlTree::is_ttl_index_valid(const int64_t & ttl_index) const
{
  if (ttl_index >= std::numeric_limits<uint8_t>::min() &&
    ttl_index <= std::numeric_limits<uint8_t>::max())
  {
    return is_ttl_index_valid(static_cast<uint8_t>(ttl_index));
  }
  return false;
}

void TtlTree::interpolate_waypoint(
  const Waypoint & p1, const Waypoint & p2, Waypoint & out,
  const double & ratio_to_p1)
{
  auto intp = [&ratio_to_p1](const double & x, const double & y) {
      return x * (1.0 - ratio_to_p1) + y * ratio_to_p1;
    };
  out.location.x = intp(p1.location.x, p2.location.x);
  out.location.y = intp(p1.location.y, p2.location.y);
  out.bank_angle = intp(p1.bank_angle, p2.bank_angle);
  out.curvature = intp(p1.curvature, p2.curvature);
  out.dist_to_sf_bwd = p1.dist_to_sf_bwd;
  out.dist_to_sf_fwd = p1.dist_to_sf_fwd;
  out.left_bound.x = intp(p1.left_bound.x, p2.left_bound.x);
  out.left_bound.y = intp(p1.left_bound.y, p2.left_bound.y);
  out.right_bound.x = intp(p1.right_bound.x, p2.right_bound.x);
  out.right_bound.y = intp(p1.right_bound.y, p2.right_bound.y);
  out.region = p1.region;
  out.target_radius = intp(p1.target_radius, p2.target_radius);
  out.target_speed = intp(p1.target_speed, p2.target_speed);
  out.target_yaw = p1.target_yaw;  // intp(p1.target_yaw, p2.target_yaw);
}

void TtlTree::get_projection(
  const Waypoint & p1, const Waypoint & p2, Waypoint & out,
  const Position & ego)
{
  CGAL::Point_2<CGAL::Cartesian<double>> p(ego.x, ego.y), projection;
  CGAL::Point_2<CGAL::Cartesian<double>> a(p1.location.x, p1.location.y), b(
    p2.location.x,
    p2.location.y);
  CGAL::Line_2<CGAL::Cartesian<double>> l(a, b);
  projection = l.projection(p);
  double total = sqrt(CGAL::squared_distance(a, b));
  double ratio_to_p1 = sqrt(CGAL::squared_distance(a, projection)) / total;
  interpolate_waypoint(p1, p2, out, ratio_to_p1);
}

PathSharedPtr TtlTree::get_interpolated_trajectory(
  const TtlIndex & ttl_idx, const Position & ego,
  const double & lookahead) const
{
  PathSharedPtr path = std::make_shared<Path>();
  auto & ttl = ttl_array_.trajectories.at(static_cast<size_t>(ttl_idx));

  std::vector<size_t> closest_2_pt;
  find_closest_waypoint_indices(ttl_idx, ego, 2, closest_2_pt);
  size_t & p1 = closest_2_pt[0], p2 = closest_2_pt[1], p3;
  if (inc_waypoint_index(ttl, p1) == p2) {
    p3 = inc_waypoint_index(ttl, p2);
  } else if (inc_waypoint_index(ttl, p2) == p1) {
    p3 = inc_waypoint_index(ttl, p1);
  } else {
    p2 = inc_waypoint_index(ttl, p1);
    p3 = inc_waypoint_index(ttl, p2);
  }

  auto & projection = path->emplace_back();
  get_projection(ttl.waypoints.at(p1), ttl.waypoints.at(p2), projection, ego);
  double remaining_distance = lookahead - get_distance(
    projection.location, ttl.waypoints.at(
      p3).location);
  path->push_back(ttl.waypoints.at(p3));

  auto prev_waypoint_itr = path->size() - 1;
  size_t prev_waypoint_index = p3;
  while (true) {
    const size_t next_waypoint_index = inc_waypoint_index(ttl, prev_waypoint_index);
    const auto & next_waypoint = ttl.waypoints.at(next_waypoint_index);
    const auto distance_to_next = get_distance(
      (*path)[prev_waypoint_itr].location,
      next_waypoint.location);
    if (distance_to_next < remaining_distance) {
      path->push_back(next_waypoint);
      remaining_distance -= distance_to_next;
    } else {
      auto & lookahead_waypoint = path->emplace_back();
      interpolate_waypoint(
        (*path)[prev_waypoint_itr], next_waypoint, lookahead_waypoint,
        remaining_distance / distance_to_next);
      break;
    }
    prev_waypoint_itr = path->size() - 1;
    prev_waypoint_index = next_waypoint_index;
  }
  return path;
}
}  // namespace race::ttl
