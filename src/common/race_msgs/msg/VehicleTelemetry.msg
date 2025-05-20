RaceControlHeader header

uint8 version_number

##################
# Race Control
##################

# Vehicle Command
VehicleCommand vehicle_cmd

# Vehicle Control Command
VehicleControlCommand control_cmd

# Vehicle Status
VehicleStatus vehicle_status

# Vehicle Kinematic State
VehicleKinematicState vks

# Push2Pass Report
Push2PassReport push2pass_rpt

# Target Trajectory Command
TargetTrajectoryCommand target_trajectory_cmd

###########
# Sensors #        
###########

# GPS
gps_msgs/GPSFix gps_fix


# IMU
sensor_msgs/Imu imu

############
# Internal #
############

# RDE Telemetry
RdeTelemetry rde_tlm

# RPP Telemetry
RppTelemetry rpp_tlm

# RVC Telemetry
RvcTelemetry rvc_tlm

############
# Race Car #
############

# Tire Report
TirePressureReport lf_tire_pressure_rpt
TirePressureReport lr_tire_pressure_rpt
TirePressureReport rf_tire_pressure_rpt
TirePressureReport rr_tire_pressure_rpt
TireTempReport tire_temp
WheelSpeedReport wheel_speed_rpt
WheelPotentiometerReport wheel_potentiometer_rpt
WheelStrainGaugeReport wheel_strain_gauge_rpt

# Engine Report
EngineReport engine_rpt

#Steering Report
SteeringReport steering_rpt

#Brake Report
BrakeReport brake_rpt

#Throttle Report
ThrottleReport throttle_rpt

# Engine Pressures Report
EnginePressuresReport engine_pressures_rpt

RideHeightReport ride_height_rpt

# DO Misc Report
MiscReport do_misc_rpt

# Fault Report
FaultReport fault_rpt

#Rest of Field Report
RaceControlRestOfField rof_rpt

# Slip Angle Report
SlipAngleReport slip_rpt

#laps
float64 lap_time

float32 long_error
float32 yaw_error
float32 commanded_yaw

##############
# Perception #
##############

# perception telemetry
PerceptionTelemetry perception_telemetry
PerceptionHealthTelemetry perception_health_telemetry

# perception stack output
vision_msgs/Detection3DArray tracked_objects