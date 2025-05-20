# Compute Slip Angle Plugin
## Purpose
This plugin computes the slip angle at the front wheel and writes it into the slip_angle_deg parameter in rvc_telemetry.
The front wheel slip angle is the angle between the front wheel velocity and the front wheel (Positive when exerting positive $F_y$ on the wheel).

## Additional Publishers and Subscribers
None

## Additional Parameters
None

## Inner Working
$\alpha_f = \delta_f - arctan(\frac{l_f \cdot \dot\psi + v_y}{v_x})$

Where $\alpha_f$ is the front wheel slip angle, $\delta_f$ is the front wheel steering angle, $l_f$ is the distance from COM to the front wheel (in the x direction only), $\dot\psi$ is the vehicle yaw rate, and $v_x$ and $v_y$ are vehicle velocities estimated at the COM.

## User Manual
Place this plugin as the last plugin to compute the value of slip_angle_deg in RvcTelemetry.msg