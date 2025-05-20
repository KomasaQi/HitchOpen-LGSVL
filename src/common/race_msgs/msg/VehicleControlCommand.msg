builtin_interfaces/Time stamp

# Accelerator pedal position from 0 to 100.
float64 accelerator_cmd

# Brake pedal position. Range varies by platform.
# IAC car uses absolute brake pressure kpi
# Less advanced platforms may use 0 to 100.
float64 brake_cmd

# Steering (radian) at tyre
float64 steering_cmd

# Speed control for supported platforms
float64 speed_cmd

# Switch between accel+brake / speed command
uint8 lon_control_type 0
uint8 LON_CONTROL_THROTTLE=0
uint8 LON_CONTROL_SPEED=1

# Desired gear between 1-6.
# TODO(dvd): Is 0 allowed as 'neutral'?
uint8 gear_cmd

# Request PURPLE_FLAG.
bool emergency_stop_cmd

# Request Push2Pass.
bool push2pass_cmd
