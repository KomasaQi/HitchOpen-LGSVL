builtin_interfaces/Time stamp

uint8 UNINITIALIZED=0
uint8 NOT_PERMITTED=1
uint8 PERMITTED=2
uint8 ACTIVE=3
uint8 EXHAUSTED=4
uint8 UNLIMITED=5
uint8 RESETTING=6

uint8 push2pass_status
# Push to pass boost remaining in seconds
float32 push2pass_budget_s
# Percentage of throttle limit that can be applied when push to pass is active
float32 push2pass_active_app_limit
