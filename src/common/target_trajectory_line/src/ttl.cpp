// Copyright 2021 Gaia Platform LLC

#include <limits>

#include "ttl.hpp"
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"

namespace race::ttl
{
struct Ellipsoid
{
  double semimajor_axis;
  double semiminor_axis;
};

Position3d to_enu(GpsPosition const & target, GpsPosition const & origin)
{
  const GeographicLib::Geocentric & earth = GeographicLib::Geocentric::WGS84();
  GeographicLib::LocalCartesian proj(origin.latitude, origin.longitude, origin.altitude, earth);
  auto out = Position3d();
  proj.Forward(target.latitude, target.longitude, target.altitude, out.x, out.y, out.z);
  return out;
}

GpsPosition to_geodetic(Position3d const & target, GpsPosition const & origin)
{
  const GeographicLib::Geocentric & earth = GeographicLib::Geocentric::WGS84();
  GeographicLib::LocalCartesian proj(origin.latitude, origin.longitude, origin.altitude, earth);
  auto out = GpsPosition();
  proj.Reverse(target.x, target.y, target.z, out.latitude, out.longitude, out.altitude);
  return out;
}

Position rotate_around(Position const & origin, Position const & position, double angle)
{
  const auto px = position.x;
  const auto py = position.y;

  const auto ox = origin.x;
  const auto oy = origin.y;

  // Rotate around origin.
  auto qx = ox + std::cos(angle) * (px - ox) - std::sin(angle) * (py - oy);
  auto qy = oy + std::sin(angle) * (px - ox) + std::cos(angle) * (py - oy);

  return Position {qx, qy};
}

double get_distance(Position const & lhs, Position const & rhs)
{
  return std::sqrt(get_distance_squared(lhs, rhs));
}

double get_distance_squared(Position const & lhs, Position const & rhs)
{
  return std::pow(lhs.x - rhs.x, 2) + std::pow(lhs.y - rhs.y, 2);
}

Position interpolate_positions(Position const & a, Position const & b, double t)
{
  const auto fraction_towards_next = std::clamp(t, 0.0, 1.0);

  const auto delta_x = b.x - a.x;
  const auto delta_y = b.y - a.y;

  auto interpolated_position = a;
  interpolated_position.x += fraction_towards_next * delta_x;
  interpolated_position.y += fraction_towards_next * delta_y;
  return interpolated_position;
}

// Returns heading in radians.
double get_heading(Position const & current, Position const & next)
{
  auto y = next.y - current.y;
  auto x = next.x - current.x;
  auto heading = std::atan2(y, x);
  return heading;
}

double get_curvature(Position const & p0, Position const & p1, Position const & p2)
{
  auto dx1 = p1.x - p0.x;
  auto dy1 = p1.y - p0.y;
  auto dx2 = p2.x - p0.x;
  auto dy2 = p2.y - p0.y;
  auto area = dx1 * dy2 - dy1 * dx2;
  auto len0 = get_distance(p0, p1);
  auto len1 = get_distance(p1, p2);
  auto len2 = get_distance(p2, p0);
  return 4.0 * area / (len0 * len1 * len2);
}

// Assumes heading is in radians.
Position get_position_from(Position const & start, double heading, double distance)
{
  auto x = start.x + distance * std::cos(heading);
  auto y = start.y + distance * std::sin(heading);
  return Position {x, y};
}

size_t inc_waypoint_index(Ttl const & ttl, size_t waypoint_index)
{
  size_t next = waypoint_index + 1;
  if (next == ttl.header.loop) {
    return 0;
  }

  return next;
}

size_t dec_waypoint_index(Ttl const & ttl, size_t waypoint_index)
{
  if (waypoint_index == 0) {
    return ttl.header.loop - 1;
  }

  return waypoint_index - 1;
}

bool is_ttl_index_valid(uint8_t ttl_index)
{
  if (ttl_index < static_cast<uint8_t>(TtlIndex::GRID_BOX)) {
    return false;
  }

  if (ttl_index >= static_cast<uint8_t>(TtlIndex::INVALID)) {
    return false;
  }

  const auto ttl = static_cast<TtlIndex>(ttl_index);
  if (ttl == TtlIndex::PIT_BOX_OUT ||
    ttl == TtlIndex::MTL4 ||
    ttl == TtlIndex::HTL2 ||
    ttl == TtlIndex::MTL3)
  {
    return true;
  }

  return false;
}

size_t find_closest_waypoint_index(Ttl const & ttl, Position const & ego)
{
  static constexpr auto MAX_ALLOWED_ERROR = 50.0;

  auto min_dist = std::numeric_limits<double>::max();
  auto found_wp = 0;

  for (size_t ii = 0; ii < ttl.header.loop; ii++) {
    auto & location = ttl.waypoints.at(ii).location;

    if (std::abs(location.x - ego.x) > MAX_ALLOWED_ERROR) {
      continue;
    }

    if (std::abs(location.y - ego.y) > MAX_ALLOWED_ERROR) {
      continue;
    }

    auto current_distance = get_distance_squared(ego, location);
    if (current_distance < min_dist) {
      min_dist = current_distance;
      found_wp = ii;
    }
  }

  return found_wp;
}

double get_cross_track_error(
  Ttl const & ttl, Position const & ego,
  uint16_t curr_waypoint_index)
{
  auto & target = ttl.waypoints[curr_waypoint_index].location;
  uint16_t prev_waypoint_index = dec_waypoint_index(ttl, curr_waypoint_index);
  auto & previous = ttl.waypoints[prev_waypoint_index].location;

  return get_cross_track_error(ego, previous, target);
}

double get_cross_track_error(
  Position const & ego,
  Position const & previous,
  Position const & target)
{
  auto C0 = ego.x;
  auto C1 = ego.y;

  auto A0 = target.x;
  auto A1 = target.y;

  auto B0 = previous.x;
  auto B1 = previous.y;

  auto cross_product = (C0 - B0) * (A1 - B1) - (A0 - B0) * (C1 - B1);
  auto base = std::sqrt((A0 - B0) * (A0 - B0) + (A1 - B1) * (A1 - B1));
  auto height = cross_product / base;
  return height;
}

bool on_track(TtlIndex index)
{
  bool on_track = (index >= TtlIndex::HTL1 && index <= TtlIndex::OTL5);
  bool returning_to_pit = (index >= TtlIndex::PIT_IN1 && index <= TtlIndex::PIT_IN5);
  return on_track || returning_to_pit;
}

bool in_pit(TtlIndex index)
{
  bool in_pit_lane = (index >= TtlIndex::PIT_IN_HIGH && index <= TtlIndex::PIT_BOX_OUT);
  bool in_pit_box = (index == TtlIndex::PIT_IN1);
  return in_pit_lane || in_pit_box;
}

bool in_pit_location(TrackLocation location)
{
  return location == TrackLocation::PIT_BOX ||
         location == TrackLocation::PIT_LANE ||
         location == TrackLocation::PIT_ROAD ||
         location == TrackLocation::PIT_CRAWL;
}

bool in_htl(TtlIndex index)
{
  return index == TtlIndex::HTL3 ||
         index == TtlIndex::HTL4 ||
         index == TtlIndex::HTL5;
}

bool in_otl(TtlIndex index)
{
  return index == TtlIndex::OTL3 ||
         index == TtlIndex::OTL4 ||
         index == TtlIndex::OTL5;
}

bool in_mtl(TtlIndex index)
{
  return index == TtlIndex::MTL3 ||
         index == TtlIndex::MTL4 ||
         index == TtlIndex::MTL5;
}

TtlIndex get_fastest_ttl(TtlIndex index)
{
  if (in_otl(index)) {return ttl::FASTEST_OTL;}
  if (in_htl(index)) {return ttl::FASTEST_HTL;}
  if (in_mtl(index)) {return ttl::FASTEST_MTL;}

  return index;
}

TtlIndex apply_otl_exception(
  TtlIndex desired_ttl,
  TtlIndex alt_ttl,
  TtlIndex excep1,
  TtlIndex excep2 /*default TtlIndex::INVALID*/)
{
  if (in_otl(desired_ttl)) {
    if (excep1 == ttl::FASTEST_OTL || excep2 == ttl::FASTEST_OTL) {
      return alt_ttl;
    }
  }
  return desired_ttl;
}

TtlIndex apply_mtl_exception(
  TtlIndex desired_ttl,
  TtlIndex alt_ttl,
  TtlIndex excep1,
  TtlIndex excep2 /*default TtlIndex::INVALID*/)
{
  if (in_mtl(desired_ttl)) {
    if (excep1 == ttl::FASTEST_MTL || excep2 == ttl::FASTEST_MTL) {
      return alt_ttl;
    }
  }
  return desired_ttl;
}

TtlIndex apply_htl_exception(
  TtlIndex desired_ttl,
  TtlIndex alt_ttl,
  TtlIndex excep1,
  TtlIndex excep2 /*default TtlIndex::INVALID*/)
{
  if (in_htl(desired_ttl)) {
    if (excep1 == ttl::FASTEST_HTL || excep2 == ttl::FASTEST_HTL) {
      return alt_ttl;
    }
  }
  return desired_ttl;
}

const char * tactic_location_name(uint8_t value)
{
  if (value == static_cast<uint8_t>(TacticLocation::STRAIGHT)) {return "STRAIGHT";}
  if (value == static_cast<uint8_t>(TacticLocation::FIRST_HALF_TURN)) {return "FIRST_HALF_TURN";}
  if (value == static_cast<uint8_t>(TacticLocation::SECOND_HALF_TURN)) {return "SECOND_HALF_TURN";}
  if (value == static_cast<uint8_t>(TacticLocation::PASSING_ZONE)) {return "PASSING_ZONE";}
  return "Unknown!";
}

const char * track_location_name(uint8_t value)
{
  if (value == static_cast<uint8_t>(TrackLocation::STRAIGHT)) {return "STRAIGHT";}
  if (value == static_cast<uint8_t>(TrackLocation::TURN_1)) {return "TURN_1";}
  if (value == static_cast<uint8_t>(TrackLocation::TURN_2)) {return "TURN_2";}
  if (value == static_cast<uint8_t>(TrackLocation::TURN_3)) {return "TURN_3";}
  if (value == static_cast<uint8_t>(TrackLocation::TURN_4)) {return "TURN_4";}
  if (value == static_cast<uint8_t>(TrackLocation::SHORT_CHUTE)) {return "SHORT_CHUTE";}
  if (value == static_cast<uint8_t>(TrackLocation::PIT_BOX)) {return "PIT_BOX";}
  if (value == static_cast<uint8_t>(TrackLocation::PIT_LANE)) {return "PIT_LANE";}
  if (value == static_cast<uint8_t>(TrackLocation::PIT_ROAD)) {return "PIT_ROAD";}
  if (value == static_cast<uint8_t>(TrackLocation::PIT_ENTRANCE)) {return "PIT_ENTRANCE";}
  if (value == static_cast<uint8_t>(TrackLocation::PIT_EXIT)) {return "PIT_EXIT";}
  if (value == static_cast<uint8_t>(TrackLocation::PIT_CRAWL)) {return "PIT_CRAWL";}
  return "Unknown!";
}

const char * ttl_name(uint8_t value)
{
  if (value == static_cast<uint8_t>(TtlIndex::GRID_BOX)) {return "GRID_BOX";}
  if (value == static_cast<uint8_t>(TtlIndex::HTL1)) {return "HTL1";}
  if (value == static_cast<uint8_t>(TtlIndex::HTL2)) {return "HTL2";}
  if (value == static_cast<uint8_t>(TtlIndex::HTL3)) {return "HTL3";}
  if (value == static_cast<uint8_t>(TtlIndex::HTL4)) {return "HTL4";}
  if (value == static_cast<uint8_t>(TtlIndex::HTL5)) {return "HTL5";}
  if (value == static_cast<uint8_t>(TtlIndex::MTL1)) {return "MTL1";}
  if (value == static_cast<uint8_t>(TtlIndex::MTL2)) {return "MTL2";}
  if (value == static_cast<uint8_t>(TtlIndex::MTL3)) {return "MTL3";}
  if (value == static_cast<uint8_t>(TtlIndex::MTL4)) {return "MTL4";}
  if (value == static_cast<uint8_t>(TtlIndex::MTL5)) {return "MTL5";}
  if (value == static_cast<uint8_t>(TtlIndex::OTL1)) {return "OTL1";}
  if (value == static_cast<uint8_t>(TtlIndex::OTL2)) {return "OTL2";}
  if (value == static_cast<uint8_t>(TtlIndex::OTL3)) {return "OTL3";}
  if (value == static_cast<uint8_t>(TtlIndex::OTL4)) {return "OTL4";}
  if (value == static_cast<uint8_t>(TtlIndex::OTL5)) {return "OTL5";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_BOX)) {return "PIT_BOX";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_IN1)) {return "PIT_IN1";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_IN2)) {return "PIT_IN2";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_IN3)) {return "PIT_IN3";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_IN4)) {return "PIT_IN4";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_IN5)) {return "PIT_IN5";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_IN_HIGH)) {return "PIT_IN_HIGH";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_IN_LOW)) {return "PIT_IN_LOW";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_BOX_IN)) {return "PIT_BOX_IN";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_OUT_HIGH)) {return "PIT_OUT_HIGH";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_OUT_LOW)) {return "PIT_OUT_LOW";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_BOX_OUT)) {return "PIT_BOX_OUT";}
  if (value == static_cast<uint8_t>(TtlIndex::START_P1)) {return "START_P1";}
  if (value == static_cast<uint8_t>(TtlIndex::START_P2)) {return "START_P2";}
  if (value == static_cast<uint8_t>(TtlIndex::START_P3)) {return "START_P3";}
  if (value == static_cast<uint8_t>(TtlIndex::START_P4)) {return "START_P4";}
  if (value == static_cast<uint8_t>(TtlIndex::TRACK_INSIDE)) {return "TRACK_INSIDE";}
  if (value == static_cast<uint8_t>(TtlIndex::TRACK_OUTSIDE)) {return "TRACK_OUTSIDE";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_INSIDE)) {return "PIT_INSIDE";}
  if (value == static_cast<uint8_t>(TtlIndex::PIT_OUTSIDE)) {return "PIT_OUTSIDE";}
  if (value == static_cast<uint8_t>(TtlIndex::INVALID)) {return "INVALID";}
  return "Unknown!";
}

}  // namespace race::ttl
