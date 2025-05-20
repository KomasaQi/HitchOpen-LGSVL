RaceControlHeader header

# Copy of the last received `flags` field from the VehicleCommand.
uint16 flags_received

# Bitfield indicating if the vehicle is meeting the requirements of
# an active flag. A bit set to 1 means the flag's requirements are met
# by the vehicle, while a bit cleared to 0 means that the flag is not active
# or the requirements of the flag are not met.
uint16 flags_met

# Bitfield of what commands are in progress. A bit set to 1
# means the command is in progress by the vehicle, while a bit cleared
# to 0 means that the command is finished or never commanded.
uint8 position_command_progress

# Bitfield of conditions related to the vehicle’s current location.
uint8 location

# Defines transitions for the Competition Team computer to communicate with Raptor.
uint8 ct_state

# Defines the Raptor’s current state machine mode.
uint8 sys_state
