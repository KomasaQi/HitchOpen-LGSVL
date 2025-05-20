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

#ifndef TTL_TREE_HPP_
#define TTL_TREE_HPP_

#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>

#include "ttl.hpp"
#include "ttl_loader.hpp"

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2 Point_d;
typedef CGAL::Search_traits_2<K> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;


namespace race::ttl
{
struct WaypointKey
{
  double x;
  double y;
  bool operator==(const WaypointKey & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct WaypointKeyHasher
{
  std::size_t operator()(const WaypointKey & k) const
  {
    return std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) << 1);
  }
};

struct TtlMapArray
{
  typedef std::unordered_map<WaypointKey, size_t, WaypointKeyHasher> TtlMap;
  std::array<TtlMap, MAX_ACTIVE_TTLS> maps;
};

struct NNTrees
{
  std::array<Tree, MAX_ACTIVE_TTLS> trees;
};

class TtlTree
{
public:
  typedef std::unique_ptr<TtlTree> UniquePtr;
  typedef std::shared_ptr<TtlTree> SharedPtr;
  typedef std::shared_ptr<const TtlTree> ConstSharedPtr;
  TtlTree();
  explicit TtlTree(const char * ttl_dir);

  void initialize(const char * ttl_dir);

  size_t find_closest_waypoint_index(const TtlIndex & ttl_idx, const Position & ego) const;
  void find_closest_waypoint_indices(
    const TtlIndex & ttl_idx, const Position & ego,
    const size_t & n, std::vector<size_t> & indices) const;
  const Ttl & get_ttl(const TtlIndex & ttl_index) const;
  const std::unordered_set<uint8_t> & get_valid_ttl_index_set() const;
  bool is_ttl_index_valid(const uint8_t & ttl_index) const;
  bool is_ttl_index_valid(const int64_t & ttl_index) const;
  PathSharedPtr get_interpolated_trajectory(
    const TtlIndex & ttl_idx, const Position & ego,
    const double & lookahead) const;
  static void get_projection(
    const Waypoint & p1, const Waypoint & p2, Waypoint & out,
    const Position & ego);
  static void interpolate_waypoint(
    const Waypoint & p1, const Waypoint & p2, Waypoint & out,
    const double & ratio_to_p1);

private:
  TtlArray ttl_array_;
  std::unordered_set<uint8_t> valid_ttl_idx_set_;
  TtlMapArray maps_;
  NNTrees trees_;
};
}  // namespace race::ttl


#endif  // TTL_TREE_HPP_
