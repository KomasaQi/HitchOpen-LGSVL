# Input Validation Plugin

## Purpose

This plugin guards the other RVC plugins against exceptions due to NaN values in input and emptry trajectory messages. It was developed with the PointOneNav Atlas GPS unit and driver in mind, which sometimes outputs NaN on startup.

## Additional Publishers and Subscribers

None

## Additional Parameters

None

## Inner-Working

The plugin checks for NaN values in

- Vehicle Kinematic State
- Target Trajectory Command
- Manual Control Command

It also checks if the trajectory is empty.

If either of the conditions above in true, it returns false in `compute_control_command`, **which causes the RVC to skip querying the rest of the plugins, and not publish any control command message.** Therefore, this plugin may not be the solution to everything, because it prevents alternative safety behaviors in the RVC (such as emergencing stopping when too many NaNs have occured). Instead, it expects the low level interface to have a timeout on the RVC output, and trigger safety related logics on its own.

A special case: if the car is in full manual mode, and the manual control command does not contain NaNs, then even if other inputs trigger the logic above, the plugin will return true in `compute_control_command`, and allow the manual command to be passed through.

## User Manual

Place this plugin as the first plugin to guard every other plugin.
