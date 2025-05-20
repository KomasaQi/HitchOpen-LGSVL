builtin_interfaces/Time stamp

#6.2.3	Appendix B3 – Vehicle Path Trajectory Update (VPTU) Message
#Each cycle the RPC will pass 6 variables to the RVC.  The RPC will determine the desired TTL, Waypoint Index, Target speed, TTL transition state, Target Transition distance, and Trajectory Line Optimizer flag. The variable names and structure are as follows.
#a.	Current_TTL_Index (Int32, index of the desired TTL within the TTL_Init_Array)
#b.	Waypoint_Index (Int32, (Index of the waypoint the Ego in currently at within the current TTL)
#c.	Target_Speed (float64, speed controlled based on table below)#
#		Value	Code Logic
#       0		Drive to TTL WP Target Speed (index 14) unless RLO_Flag is true
#		>0	Drive to the given value of Target_Speed = n
#d.	WaylineDelta (float64, desired distance from waypoint

#index of the desired TTL within the TTL_Init_Array
uint8 current_ttl_index

#Index of the waypoint the Ego in currently at within the current TTL
int32 waypoint_index

#speed controlled based on table below
#		Value	Code Logic
#       0		Drive to TTL WP Target Speed (index 14) unless RLO_Flag is true
#		>0	Drive to the given value of Target_Speed = n
float64 target_speed
# Multiplier b/w 0-1
float64 target_waypoint_scale

float64 target_gap
bool rival_car_exists
float64 rival_car_speed
float64 rival_car_gap

# requested acceleration rate to hit the target speed
# if 0 then ignore
float64 target_speed_rate

#desired distance from waypoint
float64 wayline_delta

bool push2pass_cmd

StopType stop_type

StrategyType strategy_type

# If the car is in the pit
bool in_pit_ttl
