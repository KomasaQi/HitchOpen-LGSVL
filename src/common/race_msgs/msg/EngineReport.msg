builtin_interfaces/Time stamp

bool engine_on_status
# Run switch 1 implies fuel pumps are on and primed/priming
# Run switch 0 implies pumps are off
bool engine_run_switch_status
float32 throttle_position
int8 current_gear
float32 engine_rpm
float32 vehicle_speed_kmph
# Gear shift status
# 0: Uninitialised
# 1: Ready to shift
# 2: Locked Out
# 3: Upshift in progress
# 4: Downshift in progress
int8 gear_shift_status
float32 fuel_used
float32 torque_at_wheels
