// Copyright 2021 Gaia Platform LLC
// Implements flag enums, class, and functions.

#ifndef BASE_COMMON__RACE_CONTROL_HPP_
#define BASE_COMMON__RACE_CONTROL_HPP_

#include <cstdint>

#include "race_msgs/msg/vehicle_bitfields.hpp"
#include "race_msgs/msg/vehicle_command.hpp"
#include "race_msgs/msg/vehicle_status.hpp"

#include "base_common/detail/bitmask_operators.hpp"

using race_msgs::msg::VehicleBitfields;
using race_msgs::msg::VehicleCommand;
using race_msgs::msg::VehicleStatus;

namespace race
{

// Represents the type of stop command.
enum class StopType : uint8_t
{
  NOMINAL,
  EMERGENCY,
  IMMEDIATE
};

// Represents the bit offset of the vehicle/track flags.
enum class FlagsBitfield : uint32_t
{
  NONE = 0,
  RED = (1 << VehicleBitfields::FLAGS_RED_OFFSET),
  FCY = (1 << VehicleBitfields::FLAGS_FCY_OFFSET),
  GREEN = (1 << VehicleBitfields::FLAGS_GREEN_OFFSET),
  WGREEN = (1 << VehicleBitfields::FLAGS_WGREEN_OFFSET),
  CHECKERED = (1 << VehicleBitfields::FLAGS_CHECKERED_OFFSET),
  G40 = (1 << VehicleBitfields::FLAGS_G40_OFFSET),
  G60 = (1 << VehicleBitfields::FLAGS_G60_OFFSET),
  G80 = (1 << VehicleBitfields::FLAGS_G80_OFFSET),
  G100 = (1 << VehicleBitfields::FLAGS_G100_OFFSET),
  G120 = (1 << VehicleBitfields::FLAGS_G120_OFFSET),
  G130 = (1 << VehicleBitfields::FLAGS_G130_OFFSET),
  G140 = (1 << VehicleBitfields::FLAGS_G140_OFFSET),
  G145 = (1 << VehicleBitfields::FLAGS_G145_OFFSET),
  G150 = (1 << VehicleBitfields::FLAGS_G150_OFFSET),
  G155 = (1 << VehicleBitfields::FLAGS_G155_OFFSET),
  G160 = (1 << VehicleBitfields::FLAGS_G160_OFFSET),
  G165 = (1 << VehicleBitfields::FLAGS_G165_OFFSET),
  G170 = (1 << VehicleBitfields::FLAGS_G170_OFFSET),
  G175 = (1 << VehicleBitfields::FLAGS_G175_OFFSET),
  G180 = (1 << VehicleBitfields::FLAGS_G180_OFFSET),
  G185 = (1 << VehicleBitfields::FLAGS_G185_OFFSET),
  G190 = (1 << VehicleBitfields::FLAGS_G190_OFFSET),
  ORANGE = (1 << VehicleBitfields::FLAGS_ORANGE_OFFSET),
  YELLOW = (1 << VehicleBitfields::FLAGS_YELLOW_OFFSET),
  STOP = (1 << VehicleBitfields::FLAGS_STOP_OFFSET),
  BLACK = (1 << VehicleBitfields::FLAGS_BLACK_OFFSET),
  PURPLE = (1 << VehicleBitfields::FLAGS_PURPLE_OFFSET),
  BLUE = (1 << VehicleBitfields::FLAGS_BLUE_OFFSET),
  DEFENDER = (1 << VehicleBitfields::FLAGS_DEFENDER_OFFSET),
  ATTACKER = (1 << VehicleBitfields::FLAGS_ATTACKER_OFFSET),
  EKILL = (1 << VehicleBitfields::FLAGS_EKILL_OFFSET)
};

// Represents the bit offset of the vehicle position commands.
enum class PositionCommandBitfield : uint8_t
{
  NONE = 0,
  EXIT_PIT = (1 << VehicleBitfields::POSITION_CMD_EXIT_PIT_OFFSET),
  WARMUP = (1 << VehicleBitfields::POSITION_CMD_WARMUP_OFFSET),
  FORMUP = (1 << VehicleBitfields::POSITION_CMD_FORMUP_OFFSET),
  GO_INSIDE = (1 << VehicleBitfields::POSITION_CMD_GO_INSIDE_OFFSET),
  GO_OUTSIDE = (1 << VehicleBitfields::POSITION_CMD_GO_OUTSIDE_OFFSET),
  OVERTAKE_UNDER_CAUTION = (1 << VehicleBitfields::POSITION_CMD_OVERTAKE_UNDER_CAUTION_OFFSET),
  ROLLING_START = (1 << VehicleBitfields::POSITION_CMD_ROLLING_START_OFFSET),
};

// Represents the bit offset of the vehicle location parameters.
enum class LocationBitfield : uint8_t
{
  NONE = 0,
  IN_PIT_BOX_AREA = (1 << VehicleBitfields::LOCATION_IN_PIT_BOX_AREA_OFFSET),
  IN_PIT_LANE = (1 << VehicleBitfields::LOCATION_IN_PIT_LANE_OFFSET),
  EXITING_PITS = (1 << VehicleBitfields::LOCATION_EXITING_PITS_OFFSET),
  ENTERING_PITS = (1 << VehicleBitfields::LOCATION_ENTERING_PITS_OFFSET),
  ON_TRACK = (1 << VehicleBitfields::LOCATION_ON_TRACK_OFFSET),
  LAST_LAP = (1 << VehicleBitfields::LOCATION_LAST_LAP_OFFSET),
  LAST_LAP_FINISHED = (1 << VehicleBitfields::LOCATION_LAST_LAP_FINISHED_OFFSET),
  IN_GRID_BOX_AREA = (1 << VehicleBitfields::LOCATION_IN_GRID_BOX_AREA_OFFSET)
};

template<>
struct enable_bitmask_operators<FlagsBitfield>
{
  static constexpr bool enable = true;
};

template<>
struct enable_bitmask_operators<PositionCommandBitfield>
{
  static constexpr bool enable = true;
};

template<>
struct enable_bitmask_operators<LocationBitfield>
{
  static constexpr bool enable = true;
};

template<typename E>
inline bool is_any_bit_set(E const & bitfield, E const & mask)
{
  typedef typename std::underlying_type<E>::type underlying;
  return static_cast<underlying>(bitfield & mask) != 0U;
}

template<typename E>
inline bool is_bit_set(E const & bitfield, E const & mask)
{
  return (bitfield & mask) == mask;
}

template<typename E>
inline void set_bit(E & bitfield, E const & mask)
{
  bitfield |= mask;
}

template<typename E>
inline void clear_bit(E & bitfield, E const & mask)
{
  bitfield &= ~mask;
}

template<typename E>
inline void set_bit_to(E & bitfield, E const & mask, bool is_set)
{
  if (is_set) {
    set_bit(bitfield, mask);
  } else {
    clear_bit(bitfield, mask);
  }
}

class Flags
{
public:
  Flags();

  explicit Flags(FlagsBitfield const & flags);

  operator FlagsBitfield() const
  {
    return flags_;
  }

  void set_flag(FlagsBitfield flags);

  void clear_flag(FlagsBitfield flags);

  void set_flag_to(FlagsBitfield flags, bool is_active);

  bool has_any_flag(FlagsBitfield flags) const;

  bool has_flag(FlagsBitfield flags) const;

  bool has_red_flag() const;

  bool has_fcy_flag() const;

  bool has_green_flag() const;

  // Checks if any type of green flag is set.
  //
  // Returns:
  //  - true if any green flag is set, false otherwise.
  bool has_any_green_flag() const;

  bool has_wgreen_flag() const;

  bool has_checkered_flag() const;

  bool has_g40_flag() const;

  bool has_g60_flag() const;

  bool has_g80_flag() const;

  bool has_g100_flag() const;

  bool has_g120_flag() const;

  bool has_g130_flag() const;

  bool has_g140_flag() const;

  bool has_g145_flag() const;

  bool has_g150_flag() const;

  bool has_g155_flag() const;

  bool has_g160_flag() const;

  bool has_g165_flag() const;

  bool has_g170_flag() const;

  bool has_g175_flag() const;

  bool has_g180_flag() const;

  bool has_g185_flag() const;

  bool has_g190_flag() const;

  bool has_orange_flag() const;

  bool has_yellow_flag() const;

  bool has_stop_flag() const;

  bool has_black_flag() const;

  bool has_purple_flag() const;

  bool has_blue_flag() const;

  bool has_defender_flag() const;

  bool has_attacker_flag() const;

  bool has_ekill_flag() const;

private:
  FlagsBitfield flags_;
};

void set_flags(VehicleCommand & msg, Flags const & flags);

Flags get_flags(VehicleCommand const & msg);

Flags get_flags_received(uint16_t flags_received);

uint16_t set_flags_received(Flags const & flags);

// Represents the CT state of the car (these numbers are displayed in telemetry).
enum class CtState : uint8_t
{
  UNINITIALIZED = 0,
  PWR_ON = 1,
  INITIALIZED = 2,
  ACT_TEST = 3,
  CRANK_READY = 4,
  CRANKING = 5,
  RACE_READY = 6,
  INIT_DRIVING = 7,
  CAUTION = 8,
  NOM_RACE = 9,
  COORD_STOP = 10,
  ENGINE_KILLED = 11,
  EMERGENCY_SHUTDOWN = 12,
  TOW_MODE = 14,
  INVALID = 255
};

// Represents the system state of the car (these numbers are displayed in telemetry).
enum class SysState : uint8_t
{
  NONE = 0,
  PWR_ON = 1,
  SUBSYS_CON = 2,
  ACT_TESTING = 3,
  ACT_TEST_PASSED = 4,
  WAIT_TO_CRANK = 5,
  CRANK_CHECKS_PASSED = 6,
  CRANKING = 7,
  ENG_IDLE = 8,
  DRIVING = 9,
  SHUT_ENG = 10,
  PWR_OFF = 11,
  RAPTOR_OFF = 12,
  CRANK_CHECK_INIT = 13,
  ACT_TEST_FAIL = 14,
  CRANK_CHECKS_FAIL = 15,
  EMERGENCY_SHUTDOWN = 16,
  ENG_FAILED_TO_START = 17,
  CRITICAL_LIMIT_EXCEEDED = 18,
  IGNITION_STATE = 19,
  INVALID = 255
};

// Represents the track flags (these numbers are displayed in telemetry).
enum class TrackFlag : uint8_t
{
  NONE = 0,
  RED = 3,
  FCY = 9,
  GREEN = 1,
  WGREEN = 37,
  CHECKERED = 4,
  G40 = 40,
  G60 = 60,
  G80 = 80,
  G100 = 100,
  G120 = 120,
  G130 = 130,
  G140 = 140,
  G145 = 145,
  G150 = 150,
  G155 = 155,
  G160 = 160,
  G165 = 165,
  G170 = 170,
  G175 = 175,
  G180 = 180,
  G185 = 185,
  G190 = 190,
};

// Represents the vehicle flags (these numbers are displayed in telemetry).
enum class VehicleFlag : uint8_t
{
  NONE = 0,
  ORANGE = 25,
  YELLOW = 7,
  STOP = 34,
  BLACK = 4,
  PURPLE = 32,
  BLUE = 2,
  DEFENDER = 35,
  ATTACKER = 36,
  EKILL = 33,
};

// Represents the bit offset of the sector flags.
enum class SectorFlag : uint8_t
{
  NONE = 0
};

// Checks the provided Flags object to see what track flags are set and returns
// the corresponding TrackFlag enum value.
//
// Parameters:
//  - flags: A constant reference to a Flags object containing the flags that
//           are currently set.
// Returns:
//  - The corresponding TrackFlag enum value based on the flag currently set.
TrackFlag get_track_flag_from_flags(Flags const & flags);

VehicleFlag get_vehicle_flag_from_flags(Flags const & flags);

SectorFlag get_sector_flag_from_flags(Flags const & flags);

Flags get_flags_from_track_flag(uint8_t track_flag);

Flags get_flags_from_vehicle_flag(uint8_t vehicle_flag);

Flags get_flags_from_sector_flag(uint8_t sector_flag);

}  // namespace race

#endif  // BASE_COMMON__RACE_CONTROL_HPP_
