std_msgs/Header header
string child_frame_id

# Localization outputs
# pose in frame_id
# velocity and accel in child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance velocity
geometry_msgs/AccelWithCovariance accel

# Wheel angle at tyre reported by platform
float32 front_wheel_angle_rad 0.0

# Car heading (rad) in ENU frame (bypass localization)
float32 car_yaw 0.0

# Velocity heading (rad) in ENU frame (bypass localization)
float32 velocity_yaw 0.0

# Car yaw rate (rad/s) (bypass localization)
float32 car_yaw_rate 0.0

# Velocity yaw rate (rad/s) (bypass localization)
float32 velocity_yaw_rate 0.0

# Scalar speed (m/s) (typically reported by GPS position diff)
float32 speed_mps 0.0

# Indication of sensor failures
uint8 source_status_code 0

# Bitfield offsets of source status code
uint8 SOURCE_POS_BAD=0
uint8 SOURCE_SPEED_BAD=1
uint8 SOURCE_ACCEL_BAD=2
uint8 SOURCE_WHEEL_ANGLE_BAD=3
