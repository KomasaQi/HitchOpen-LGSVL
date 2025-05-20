builtin_interfaces/Time stamp

###########
# Outputs #
###########

# Accelerator pedal position from 0 to 100.
float32 throttle_cmd

# Brake pedal position from 0 to 100.
float32 brake_cmd

# Steering wheel position from -100 to 100.
float32 steering_cmd

#################
# Path Planning #
#################

# Commanded wayline delta from RDE, usually used in warm-up.
float32 wayline_delta

# Distance of lookahead (m)
float32 lookahead_distance

# Distance to projection on TTL
float32 lateral_error

# Index of lookahead waypoint
float32 lookahead_index

# Orthogonal projection of car position onto TTL
float32 projection_x
float32 projection_y

# Lookahead position
float32 lookahead_x
float32 lookahead_y

###########
# Control #
###########

# Target speed is equal to speed limit most of the time. 
# If the lateral error is huge, it is the current speed (don't accelerate).
# If timeout or emergency occurs, it is zero.
float32 target_speed

float32 target_lon_acc

float32 target_lat_acc

# Current speed from localization
float32 current_speed

# Speed limit commanded by RDE
float32 speed_limit

float32 slip_angle_deg

###########
# Gearbox #
###########

# Timestamp (sec) of the last gearshift request
float32 last_gearshift_time_sec

# After a gearshift request, there is a configurable amount of cooldown.
bool is_in_gearshift_cooldown

# Gear command
int8 gear_command

# If the MPC is enabled
bool mpc_enabled

###############
# Diagnostics #
###############

# Something timed out in the last cycle
bool has_timeout

# Some watchdog has not been reset even once
bool has_uninitialized_clock

# Name of timed out stuffs
string timeout_clocks

# Name of uninitialized stuffs
string uninitialized_clocks

# Time of computation of the last cycle
float32 cycle_sec