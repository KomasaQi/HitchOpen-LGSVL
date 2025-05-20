// Copyright 2021 Gaia Platform LLC
//
// Defines getters, setters and management for the flags.

#include "base_common/race_control.hpp"

namespace race
{

Flags::Flags()
: flags_{FlagsBitfield::NONE}
{
}

Flags::Flags(FlagsBitfield const & flags)
: flags_{flags}
{
}

void Flags::set_flag(FlagsBitfield flags)
{
  set_bit(flags_, flags);
}

void Flags::clear_flag(FlagsBitfield flags)
{
  clear_bit(flags_, flags);
}

void Flags::set_flag_to(FlagsBitfield flags, bool is_active)
{
  set_bit_to(flags_, flags, is_active);
}

bool Flags::has_any_flag(FlagsBitfield flags) const
{
  return is_any_bit_set(flags_, flags);
}

bool Flags::has_flag(FlagsBitfield flags) const
{
  return is_bit_set(flags_, flags);
}

bool Flags::has_red_flag() const
{
  return has_flag(FlagsBitfield::RED);
}

bool Flags::has_fcy_flag() const
{
  return has_flag(FlagsBitfield::FCY);
}

bool Flags::has_green_flag() const
{
  return has_flag(FlagsBitfield::GREEN);
}

bool Flags::has_any_green_flag() const
{
  return has_green_flag() || has_wgreen_flag() || has_g40_flag() || has_g60_flag() ||
         has_g80_flag() || has_g100_flag() ||
         has_g120_flag() || has_g130_flag() || has_g140_flag() || has_g145_flag() ||
         has_g150_flag() ||
         has_g155_flag() || has_g160_flag() || has_g165_flag() || has_g170_flag() ||
         has_g175_flag() ||
         has_g180_flag() || has_g185_flag() || has_g190_flag();
}

bool Flags::has_wgreen_flag() const
{
  return has_flag(FlagsBitfield::WGREEN);
}

bool Flags::has_checkered_flag() const
{
  return has_flag(FlagsBitfield::CHECKERED);
}

bool Flags::has_g40_flag() const
{
  return has_flag(FlagsBitfield::G40);
}

bool Flags::has_g60_flag() const
{
  return has_flag(FlagsBitfield::G60);
}

bool Flags::has_g80_flag() const
{
  return has_flag(FlagsBitfield::G80);
}

bool Flags::has_g100_flag() const
{
  return has_flag(FlagsBitfield::G100);
}

bool Flags::has_g120_flag() const
{
  return has_flag(FlagsBitfield::G120);
}

bool Flags::has_g130_flag() const
{
  return has_flag(FlagsBitfield::G130);
}

bool Flags::has_g140_flag() const
{
  return has_flag(FlagsBitfield::G140);
}

bool Flags::has_g145_flag() const
{
  return has_flag(FlagsBitfield::G145);
}

bool Flags::has_g150_flag() const
{
  return has_flag(FlagsBitfield::G150);
}

bool Flags::has_g155_flag() const
{
  return has_flag(FlagsBitfield::G155);
}

bool Flags::has_g160_flag() const
{
  return has_flag(FlagsBitfield::G160);
}

bool Flags::has_g165_flag() const
{
  return has_flag(FlagsBitfield::G165);
}

bool Flags::has_g170_flag() const
{
  return has_flag(FlagsBitfield::G170);
}

bool Flags::has_g175_flag() const
{
  return has_flag(FlagsBitfield::G175);
}

bool Flags::has_g180_flag() const
{
  return has_flag(FlagsBitfield::G180);
}

bool Flags::has_g185_flag() const
{
  return has_flag(FlagsBitfield::G185);
}

bool Flags::has_g190_flag() const
{
  return has_flag(FlagsBitfield::G190);
}

bool Flags::has_orange_flag() const
{
  return has_flag(FlagsBitfield::ORANGE);
}

bool Flags::has_yellow_flag() const
{
  return has_flag(FlagsBitfield::YELLOW);
}

bool Flags::has_stop_flag() const
{
  return has_flag(FlagsBitfield::STOP);
}

bool Flags::has_black_flag() const
{
  return has_flag(FlagsBitfield::BLACK);
}

bool Flags::has_purple_flag() const
{
  return has_flag(FlagsBitfield::PURPLE);
}

bool Flags::has_blue_flag() const
{
  return has_flag(FlagsBitfield::BLUE);
}

bool Flags::has_defender_flag() const
{
  return has_flag(FlagsBitfield::DEFENDER);
}

bool Flags::has_attacker_flag() const
{
  return has_flag(FlagsBitfield::ATTACKER);
}

bool Flags::has_ekill_flag() const
{
  return has_flag(FlagsBitfield::EKILL);
}

void set_flags(VehicleCommand & msg, Flags const & flags)
{
  msg.flags = static_cast<uint32_t>(static_cast<FlagsBitfield>(flags));
}

Flags get_flags(VehicleCommand const & msg)
{
  return Flags(static_cast<FlagsBitfield>(msg.flags));
}

Flags get_flags_received(uint16_t flags_received)
{
  return Flags(static_cast<FlagsBitfield>(flags_received));
}

uint16_t set_flags_received(Flags const & flags)
{
  return static_cast<uint16_t>(static_cast<FlagsBitfield>(flags));
}

TrackFlag get_track_flag_from_flags(Flags const & flags)
{
  if (flags.has_red_flag()) {
    return TrackFlag::RED;
  } else if (flags.has_fcy_flag()) {
    return TrackFlag::FCY;
  } else if (flags.has_green_flag()) {
    return TrackFlag::GREEN;
  } else if (flags.has_wgreen_flag()) {
    return TrackFlag::WGREEN;
  } else if (flags.has_checkered_flag()) {
    return TrackFlag::CHECKERED;
  } else if (flags.has_g40_flag()) {
    return TrackFlag::G40;
  } else if (flags.has_g60_flag()) {
    return TrackFlag::G60;
  } else if (flags.has_g80_flag()) {
    return TrackFlag::G80;
  } else if (flags.has_g100_flag()) {
    return TrackFlag::G100;
  } else if (flags.has_g120_flag()) {
    return TrackFlag::G120;
  } else if (flags.has_g130_flag()) {
    return TrackFlag::G130;
  } else if (flags.has_g140_flag()) {
    return TrackFlag::G140;
  } else if (flags.has_g145_flag()) {
    return TrackFlag::G145;
  } else if (flags.has_g150_flag()) {
    return TrackFlag::G150;
  } else if (flags.has_g155_flag()) {
    return TrackFlag::G155;
  } else if (flags.has_g160_flag()) {
    return TrackFlag::G160;
  } else if (flags.has_g165_flag()) {
    return TrackFlag::G165;
  } else if (flags.has_g170_flag()) {
    return TrackFlag::G170;
  } else if (flags.has_g175_flag()) {
    return TrackFlag::G175;
  } else if (flags.has_g180_flag()) {
    return TrackFlag::G180;
  } else if (flags.has_g185_flag()) {
    return TrackFlag::G185;
  } else if (flags.has_g190_flag()) {
    return TrackFlag::G190;
  }

  return TrackFlag::NONE;
}


// Checks the provided Flags object to see what vehicle flags are set and
// returns the corresponding TrackFlag enum value.
//
// Parameters:
// -  flags: A constant reference to a Flags object containing the flags that
//           are currently set.
// Returns:
//  - The corresponding VehicleFlag enum value based on the flag currently set.
VehicleFlag get_vehicle_flag_from_flags(Flags const & flags)
{
  if (flags.has_orange_flag()) {
    return VehicleFlag::ORANGE;
  } else if (flags.has_yellow_flag()) {
    return VehicleFlag::YELLOW;
  } else if (flags.has_stop_flag()) {
    return VehicleFlag::STOP;
  } else if (flags.has_black_flag()) {
    return VehicleFlag::BLACK;
  } else if (flags.has_purple_flag()) {
    return VehicleFlag::PURPLE;
  } else if (flags.has_black_flag()) {
    return VehicleFlag::BLUE;
  } else if (flags.has_defender_flag()) {
    return VehicleFlag::DEFENDER;
  } else if (flags.has_attacker_flag()) {
    return VehicleFlag::ATTACKER;
  } else if (flags.has_ekill_flag()) {
    return VehicleFlag::EKILL;
  }

  return VehicleFlag::NONE;
}

SectorFlag get_sector_flag_from_flags(Flags const & flags)
{
  // Not used in IAC at the moment
  (void)flags;
  return SectorFlag::NONE;
}

Flags get_flags_from_track_flag(uint8_t track_flag)
{
  if (track_flag == static_cast<uint8_t>(TrackFlag::RED)) {
    return Flags(FlagsBitfield::RED);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::FCY)) {
    return Flags(FlagsBitfield::FCY);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::GREEN)) {
    return Flags(FlagsBitfield::GREEN);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::WGREEN)) {
    return Flags(FlagsBitfield::WGREEN);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::CHECKERED)) {
    return Flags(FlagsBitfield::CHECKERED);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G40)) {
    return Flags(FlagsBitfield::G40);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G60)) {
    return Flags(FlagsBitfield::G60);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G80)) {
    return Flags(FlagsBitfield::G80);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G100)) {
    return Flags(FlagsBitfield::G100);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G120)) {
    return Flags(FlagsBitfield::G120);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G130)) {
    return Flags(FlagsBitfield::G130);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G140)) {
    return Flags(FlagsBitfield::G140);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G145)) {
    return Flags(FlagsBitfield::G145);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G150)) {
    return Flags(FlagsBitfield::G150);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G155)) {
    return Flags(FlagsBitfield::G155);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G160)) {
    return Flags(FlagsBitfield::G160);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G165)) {
    return Flags(FlagsBitfield::G165);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G170)) {
    return Flags(FlagsBitfield::G170);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G175)) {
    return Flags(FlagsBitfield::G175);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G180)) {
    return Flags(FlagsBitfield::G180);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G185)) {
    return Flags(FlagsBitfield::G185);
  } else if (track_flag == static_cast<uint8_t>(TrackFlag::G190)) {
    return Flags(FlagsBitfield::G190);
  }

  return Flags(FlagsBitfield::NONE);
}

Flags get_flags_from_vehicle_flag(uint8_t vehicle_flag)
{
  if (vehicle_flag == static_cast<uint8_t>(VehicleFlag::ORANGE)) {
    return Flags(FlagsBitfield::ORANGE);
  } else if (vehicle_flag == static_cast<uint8_t>(VehicleFlag::YELLOW)) {
    return Flags(FlagsBitfield::YELLOW);
  } else if (vehicle_flag == static_cast<uint8_t>(VehicleFlag::STOP)) {
    return Flags(FlagsBitfield::STOP);
  } else if (vehicle_flag == static_cast<uint8_t>(VehicleFlag::BLACK)) {
    return Flags(FlagsBitfield::BLACK);
  } else if (vehicle_flag == static_cast<uint8_t>(VehicleFlag::PURPLE)) {
    return Flags(FlagsBitfield::PURPLE);
  } else if (vehicle_flag == static_cast<uint8_t>(VehicleFlag::BLUE)) {
    return Flags(FlagsBitfield::BLUE);
  } else if (vehicle_flag == static_cast<uint8_t>(VehicleFlag::DEFENDER)) {
    return Flags(FlagsBitfield::DEFENDER);
  } else if (vehicle_flag == static_cast<uint8_t>(VehicleFlag::ATTACKER)) {
    return Flags(FlagsBitfield::ATTACKER);
  } else if (vehicle_flag == static_cast<uint8_t>(VehicleFlag::EKILL)) {
    return Flags(FlagsBitfield::EKILL);
  }

  return Flags(FlagsBitfield::NONE);
}

Flags get_flags_from_sector_flag(uint8_t sector_flag)
{
  // Not used in IAC at the moment
  (void)sector_flag;
  return Flags(FlagsBitfield::NONE);
}

}  // namespace race
