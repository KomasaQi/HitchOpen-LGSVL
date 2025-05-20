# Compliance Plugin

## Purpose

This plugin sanity checks the output of RVC. It checks for NaNs, out-of-bound values, and regulation compliance.

## Additional Publishers and Subscribers

None

## Additional Parameters

Parameter namespace: `compliance`

| Parameter Name             | Type   | Default Value | Description                                                               |
|----------------------------|--------|---------------|---------------------------------------------------------------------------|
| `max_throttle_cmd`         | double | 100.0         | Max throttle value to clip to in throttle control mode.                   |
| `max_brake_cmd`            | double | 100.0         | Max brake value to clip to in throttle control mode.                      |
| `max_speed_cmd`            | double | 100.0         | Max speed value to clip to in speed control mode.                         |
| `min_brake_cmd_under_stop` | double | 10.0          | If a stopping is commanded, this is the minimum brake command to clip to. |

Additionally, it uses `rvc.max_front_wheel_angle_rad` to clip the steering output.

## Inner-Working

If `lon_control_type` is `LON_CONTROL_THROTTLE`, the plugin checks throttle, brake, and steering output.

If `lon_control_type` is `LON_CONTROL_SPEED`, the plugin checks speed and steering output.

If `lon_control_type` is non of the defined types above, the plugin checks steering output, and prints error messages to terminal.

For every variable it checks, it overrides NaN values to zero, and apply clipping to out of bound values. It prints error messages to terminal in these situations.

## User Manual

Place this plugin as the last plugin to apply a last sanity check.
