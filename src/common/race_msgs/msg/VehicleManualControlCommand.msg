RaceControlHeader header

# While true, RVC will pass through manual commands instead of autonomous outputs.
bool use_manual_cmd
# While true, RVC will limit maximum throttle to the throttle position on the manual command.
bool limit_auto_throttle
# If true, the joystick multiplexer will give control to the joystick that sends it
bool take_control

VehicleControlCommand vehicle_control_command
