# Manual Override Plugin

## Purpose

This plugin mainly serves as a multiplexer for manual and autonomous control signals.

| Mode                                       | `use_manual_cmd` | `limit_auto_throttle` | Description                                         |
|--------------------------------------------|------------------|-----------------------|-----------------------------------------------------|
| Full manual                                | True             | Any                   | Full manual control if needed                       |
| Auto steering + auto throttle with deadman | False            | True                  | Auto throttle is scaled by manual throttle position |
| Auto steering + auto throttle              | False            | False                 | In race where interaction is not allowed            |

In the sheet above, `use_manual_cmd` and `limit_auto_throttle` are flags in `VehicleManualControlCommand` message from RDE, who determines the operation mode.


## Additional Publishers and Subscribers

None

## Additional Parameters

Parameter namespace: `override`.


| Parameter Name                | Type   | Default Value | Description                                                                                        |
|-------------------------------|--------|---------------|----------------------------------------------------------------------------------------------------|
| `throtte_threshold`           | double | None          | Manual throttle above this value will trigger override.                                            |
| `brake_threshold`             | double | None          | Manual brake above this value will trigger override.                                               |
| `steer_threshold_deg`         | double | None          | Manual steering above this value will trigger override.                                            |
| `max_manual_speed_mps`        | double | None          | When the vehicle is faster than this value in manual mode, throttle will be set to zero.           |
| `steer_decay.start_speed_mps` | double | None          | When the vehicle is faster than this value, the range of manual steering override start to shrink. |
| `steer_decay.end_speed_mps`   | double | None          | When the vehicle is faster than this value, manual steering override is disabled.                  |

## Inner-Working

### Limited manual steering override range in high speed

When the vehicle speed is slower than `steer_decay.start_speed_mps`, manual steering override is unscaled.

When the vehicle speed is between `steer_decay.start_speed_mps` and `steer_decay.end_speed_mps`, manual steering override is scaled by the function $y=-0.5\sin(x) + 0.5,x\in[-\pi/2, \pi/2]$, such that the scale smoothly transitions from 1.0 to 0.0 at the end.

When the vehicle speed is faster than `steer_decay.end_speed_mps`, manual steering override is disabled and will have no effect on the steering at all. It is expected that any hazardous situation above this speed can only be handled through slowing down, such as leaving off the throttle deadman trigger in limit-auto-throttle mode, or overriding the brake.

## User Manual

The joystick operator should understand the expected behavior of the plugin and be aware of the stratergies against various hazards under different speeds.

In general, cases where manual intervention is needed are:

- "Kickstart": e.g. driving out of the pit, or manually moving the vehicle to a line.
- Uncertain situations: e.g. flag timeout.
- Hazardous situations: e.g. Imminent crash.

In low speed (Below 20 mps for IAC vehicle), the joystick operator should feel free to override any control output at any time. Be aware of the delay in the communication.

In mid speed (20-35 mps for IAC vehicle), steering override should be avoided. In hazardous situations, brake override should be gentle as to not lock the wheels. In uncertain situations (such as a flag timeout), simply leaving off the throttle deadman trigger is the ideal way of slowing down.

In high speed (above 35 mps for IAC vehicle), steering override should be forbidden. Leaving off the deadman trigger is always the preferred way to manually slow down the vehicle. Brake override should only be done when the vehicle has slowed down to mid speed range, or the vehicle has already lost control.

In mid and high speed scenarios, the joystick operator should keep away from the steering stick to avoid accidental movement. For example, keep the thumb below the joystick.

If emergency shutdown is required, it is advised to do so after the car has come to a stop (press both shoulder buttons).