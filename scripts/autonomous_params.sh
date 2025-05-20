ros2 param set /race_decision_engine_node use_params true
ros2 param set /race_decision_engine_node use_perception false
# ros2 param set /basestation_race_control_node track_flag "fcy"
ros2 param set /basestation_race_control_node track_flag "green"
# ros2 param set /basestation_race_control_node track_flag "g80"
ros2 param set /basestation_race_control_node vehicle_flags ["none"]
ros2 param set /publish_dummy_joystick_control limit_auto_throttle false
ros2 param set /publish_dummy_joystick_control use_manual_cmd false
ros2 param set /race_decision_engine_node speed_limit.green 20.0 # Just an example of how to set speed
ros2 param set /race_decision_engine_node ttls.race_ttl_index 15