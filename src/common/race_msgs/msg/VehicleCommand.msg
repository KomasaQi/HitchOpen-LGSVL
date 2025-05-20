RaceControlHeader header

# Bitfield indicates which flags are currently active for the vehicle. A bit
# set to 1 means the flag is active for that vehicle, while a bit cleared to 0
# means the flag is not active for that vehicle
uint32 flags

# Bitfield indicates which position commands are currently active for the
# vehicle. A bit set to 1 means the command is active for that vehicle, while
# a bit cleared to 0 means there is no command of that type for the vehicle
uint8 position_command

# The vehicle’s current position in the race, e.g. 2nd place, as indicated
# by the value 1 through 255.
uint8 track_position

# Number of laps completed by vehicle.
uint8 laps

# Fraction of the current lap completed by the vehicle in 0.1% increments.
# Each 0.1% increment is approximately 4 meters at IMS.
uint16 laps_fraction

float64 round_target_speed