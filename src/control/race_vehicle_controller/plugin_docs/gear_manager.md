# Gear Manager Plugin

## Purpose

This plugin is designed with the IAC vehicle in mind. It manages the target gear command from the RVC and shifts the car into the right gear at the right RPM.

## Additional Publishers and Subscribers

| Subscription Type            | Default Topic   | Description                                     |
|------------------------------|-----------------|-------------------------------------------------|
| `race_msgs/msg/EngineReport` | `engine_report` | Engine report including RPM reported by vehicle |

## Additional Parameters

Parameter namespace: `gear_manager`

| Parameter Name         | Type           | Default Value | Description                                          |
|------------------------|----------------|---------------|------------------------------------------------------|
| `gear_numbers`         | list\<double\> | Empty         | What are the gear numbers? (IAC uses 1-6)            |
| `min_rpms`             | list\<double\> | Empty         | Downshift RPM at each gear (first number is ignored) |
| `max_rpms`             | list\<double\> | Empty         | Upshift RPM at each gear (last number is ignored)    |
| `start_gear`           | double         | 1             | Default gear                                         |
| `gear_change_wait_sec` | double         | 0.5           | Minimum interval between gearshifts                  |

## Inner-Working

As a new engine report comes in, the plugin checks if the RPM is too high or too low for the current gear, and updates its desired gear command. The number is then picked up by RVC in `compute_control_command` cycle. Then the plugin waits for the minimum interval defined above, and checks for gear change status. In case of success, it will update its internal gear representation to the new gear number. If failed, it will reset its internal gear representation to the true current gear and attempt shifting again.

Here are the IAC ECU's gear shift status definitions:

| Gear Shift Status | Meaning       | Happens When                      |
|-------------------|---------------|-----------------------------------|
| 0                 | Uninitialized | Gearshift hasn't been attempted   |
| 1                 | Available     | Gearshift completed and available |
| 2                 | Locked Out    | Gearshift failing                 |
| 3                 | Upshifting    | Upshifting                        |
| 4                 | Downshifting  | Downshifting                      |

Before the first gear shift, due to the IAC ECU software, a gear shift status of LOCKEDOUT could also mean that shifting is available.

## User Manual

The order of this plugin doesn't matter.

User should be fully informed of any latest IAC ECU changes and update the plugin accordingly.

IAC maintains a spreadsheet of gear shift ranges. Do not configure minimum and maximum RPM parameters beyond this range.

It is possible, for example, when the car upshifts, the RPM immediately falls below the minimum RPM of the higher gear, which sends the vehicle back to the lower gear, and repeats the process. You will hear the engine going "high-low-high-low" at the minimum gear shift interval defined above. You will also see gear number fluctuating in telemetry. In this case, increase the upshift RPM of the lower gear, or decrease the minimum RPM of the higher gear.
