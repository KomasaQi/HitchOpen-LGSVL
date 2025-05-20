// Copyright 2021 Gaia Platform LLC

#ifndef TTL_HPP_
#define TTL_HPP_

#include <array>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <memory>

// This file describes the Target Trajectory Line structures.
// All of this data is read-only after initialization.
namespace race::ttl
{

struct GpsPosition
{
  double latitude;
  double longitude;
  double altitude;
};

struct Position3d
{
  double x;
  double y;
  double z;
};

struct Position
{
  double x;
  double y;

  friend bool operator==(const Position & lhs, const Position & rhs);

  constexpr Position()
  : x{0.0}, y{0.0}
  {}

  constexpr Position(double new_x, double new_y)
  : x{new_x}, y{new_y}
  {}

  explicit constexpr Position(const Position3d & other)
  : x{other.x}, y{other.y}
  {}
};

inline bool operator==(const Position & lhs, const Position & rhs)
{
  return lhs.x == rhs.x &&
         lhs.y == rhs.y;
}

// Location on track
enum class TacticLocation : uint8_t
{
  STRAIGHT,
  FIRST_HALF_TURN,
  SECOND_HALF_TURN,
  PASSING_ZONE
};

// Order around the track is:
// straight
// turn_1
// short_chute
// turn_2
// straight
// turn_3
// short_chute
// turn_4
// back to beginning.
enum class TrackLocation : uint8_t
{
  STRAIGHT,
  TURN_1,
  TURN_2,
  TURN_3,
  TURN_4,
  SHORT_CHUTE,
  PIT_BOX,
  PIT_LANE,
  PIT_ROAD,
  PIT_ENTRANCE,
  PIT_EXIT,
  PIT_CRAWL
};

using SteeringLocation = size_t;
using LaneNumber = uint8_t;

// This is the TTL_STATE field in the Vehicle Path Trajectory Update message.
enum class TtlState : uint8_t
{
  TRANS,
  STEADY
};

// Note that only the TTLs with comments
// by them are suppported.  These are supported
// both for LOR and IMS tracks.
enum class TtlIndex : uint8_t
{
  GRID_BOX,
  HTL1,
  HTL2,
  HTL3,  // 3: High Trajectory Lines
  HTL4,  //
  HTL5,  //
  MTL1,
  MTL2,
  MTL3,  // 8: Mid Trajectory Lines
  MTL4,  //
  MTL5,  //
  OTL1,
  OTL2,
  OTL3,  // 13: Optimum Trajectory Lines
  OTL4,  //
  OTL5,  //
  PIT_BOX,
  PIT_IN1,
  PIT_IN2,
  PIT_IN3,  // 19: Pit in from these lanes
  PIT_IN4,  //
  PIT_IN5,  //
  PIT_IN_HIGH,   // 22:  High speed pit lane in
  PIT_IN_LOW,    // Low speed pit lane in
  PIT_BOX_IN,    // PIT_LANE_LOW to the PIT_BOX
  PIT_OUT_HIGH,  // High speed pit lane out
  PIT_OUT_LOW,   // Low speed pit lane out
  PIT_BOX_OUT,   // PIT_BOX to PIT_OUT_LOW
  START_P1,
  START_P2,
  START_P3,
  START_P4,
  TRACK_INSIDE,
  TRACK_OUTSIDE,
  PIT_INSIDE,
  PIT_OUTSIDE,
  INVALID,
};

enum TtlColumn
{
  X = 0,
  Y = 1,
  Z = 2,
  TARGET_YAW = 3,
  TARGET_SPEED = 4,
  CURVATURE = 5,
  DIST_TO_SF_BWD = 6,
  DIST_TO_SF_FWD = 7,
  REGION = 8,
  LEFT_BOUND_X = 9,
  LEFT_BOUND_Y = 10,
  RIGHT_BOUND_X = 11,
  RIGHT_BOUND_Y = 12,
  BANK_ANGLE = 13,
  NORMAL_X = 14,
  NORMAL_Y = 15,
};

// Loaded from waypoint 0 array
struct TtlHeader
{
  std::string ttl_name;
  TtlIndex number;
  uint32_t loop;
  double total_distance;
  GpsPosition origin;
};

// Loaded for waypoints 1-999

// TODO(dax): in original spec, this was represented as double[22];
// However, having struct names will make things more readable
// and we can map the data in the .csv file to this structure
struct Waypoint
{
  Position location;
  double normal_x;
  double normal_y;
  double target_yaw;
  double target_speed;
  double target_radius;
  double curvature;
  double dist_to_sf_bwd;
  double dist_to_sf_fwd;
  TrackLocation region;
  Position left_bound;
  Position right_bound;
  double bank_angle;
};

typedef std::vector<Waypoint> Path;
typedef std::shared_ptr<Path> PathSharedPtr;

const uint16_t MAX_WAYPOINTS = 5000;

// Target Trajectory Line
struct Ttl
{
  TtlHeader header;
  std::array<Waypoint, MAX_WAYPOINTS> waypoints;
};

//
// Heirarchy of fastest lanes
//
//
static constexpr uint8_t NUM_LANES = 5;

static constexpr std::array<uint8_t, NUM_LANES + 1> OTL_TO_LANE_MAP {
  0,   // unused
  5,   // OTL1 -> rank 5
  3,   // OTL2 -> rank 3
  4,   // OTL3 -> rank 4
  2,   // OTL4 -> rank 2
  1    // OTL5 -> rank 1
};

// TODO(dax) : Make these configurable
static constexpr TtlIndex FASTEST_OTL = TtlIndex::OTL1;
static constexpr TtlIndex FASTEST_MTL = TtlIndex::MTL3;
static constexpr TtlIndex FASTEST_HTL = TtlIndex::HTL3;

// We support OTLs 1, 3, 5
// TODO(dax): if we are in lane 2 do we want to got to 1 or 3?
static constexpr std::array<TtlIndex, NUM_LANES + 1> LANE_TO_OTL_MAP {
  TtlIndex::INVALID,
  TtlIndex::OTL1,
  TtlIndex::OTL3,
  TtlIndex::OTL3,
  TtlIndex::OTL5,
  TtlIndex::OTL5
};

// We only support MTLs 1, 3
static constexpr std::array<TtlIndex, NUM_LANES + 1> LANE_TO_MTL_MAP {
  TtlIndex::INVALID,
  TtlIndex::MTL1,
  TtlIndex::MTL3,
  TtlIndex::MTL3,
  TtlIndex::OTL3,
  TtlIndex::OTL5
};

// We only support HTL 1
static constexpr std::array<TtlIndex, NUM_LANES + 1> LANE_TO_HTL_MAP {
  TtlIndex::INVALID,
  TtlIndex::HTL1,
  TtlIndex::MTL3,
  TtlIndex::MTL3,
  TtlIndex::MTL3,
  TtlIndex::OTL5
};

bool on_track(TtlIndex index);

bool in_pit(TtlIndex index);

bool in_pit_lane(TtlIndex index);

bool in_pit_location(TrackLocation location);

bool in_htl(TtlIndex index);

bool in_otl(TtlIndex index);

bool in_mtl(TtlIndex index);

TtlIndex get_fastest_ttl(TtlIndex index);

TtlIndex apply_otl_exception(
  TtlIndex desired_ttl,
  TtlIndex alt_ttl,
  TtlIndex excep1,
  TtlIndex excep2 = TtlIndex::INVALID);

TtlIndex apply_mtl_exception(
  TtlIndex desired_ttl,
  TtlIndex alt_ttl,
  TtlIndex excep1,
  TtlIndex excep2 = TtlIndex::INVALID);

TtlIndex apply_htl_exception(
  TtlIndex desired_ttl,
  TtlIndex alt_ttl,
  TtlIndex excep1,
  TtlIndex excep2 = TtlIndex::INVALID);

const uint8_t MAX_ACTIVE_TTLS = 36;

// Go back this many waypoint indexes when switching
// TTLs to search for the corresponding waypoint.
static constexpr uint8_t NUM_TTL_WAYPOINTS_TO_SEARCH = 20;

// Coordinates of the finish line to count laps.  This information
// may be used to simulate tire degradation over the course of the race.
// Unused presently as we are just going to use the distance travelled by the
// Ego to model tire wear

// Statistics on total distance race distance covered and per lap
static constexpr uint32_t LAP_DISTANCE = 4050;

Position3d to_enu(GpsPosition const & target, GpsPosition const & origin);
GpsPosition to_geodetic(Position3d const & target, GpsPosition const & origin);

Position rotate_around(Position const & origin, Position const & position, double angle);

double get_distance(Position const & lhs, Position const & rhs);

double get_distance_squared(Position const & lhs, Position const & rhs);

double get_heading(Position const & current, Position const & next);

double get_curvature(Position const & p1, Position const & p2, Position const & p3);

Position get_position_from(Position const & start, double heading, double distance);

size_t inc_waypoint_index(Ttl const & ttl, size_t waypoint_index);

size_t dec_waypoint_index(Ttl const & ttl, size_t waypoint_index);

bool is_ttl_index_valid(uint8_t ttl_index);

const char * tactic_location_name(uint8_t location);

const char * track_location_name(uint8_t location);

const char * ttl_name(uint8_t ttl);

size_t find_closest_waypoint_index(Ttl const & ttl, Position const & ego);

Position interpolate_positions(Position const & a, Position const & b, double t);

double get_cross_track_error(
  Ttl const & ttl,
  Position const & ego,
  uint16_t curr_waypoint_index);

double get_cross_track_error(
  Position const & ego,
  Position const & previous,
  Position const & target);

struct TtlArray
{
  std::array<Ttl, MAX_ACTIVE_TTLS> trajectories;
};

}  // namespace race::ttl

#endif  // TTL_HPP_
