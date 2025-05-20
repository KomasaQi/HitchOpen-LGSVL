uint8 STOP_TYPE_NOMINAL=0
uint8 STOP_TYPE_SAFE=1
uint8 STOP_TYPE_IMMEDIATE=2
uint8 STOP_TYPE_EMERGENCY=3

#controls behavior when target speed is slower than current speed
#		Value	Code Logic
#       0		Decelerate using nominal speed rate limit.
#       1		Decelerate using nominal speed rate limit and stop
#       2		Decelerate as dynamically feasibly comming to a stop immediately
#		3       Stop immediately. Could require locking brakes
uint8 stop_type