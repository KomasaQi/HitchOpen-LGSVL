# Race Vehicle Controller (RVC)

## Purpose of Package

The Race Vehicle Controller (RVC), upon receiving a trajectory commanded by the Race Decision Engine (RDE), calculates the control command in terms of throttle, brake and steering that would keep the vehicle on the trajectory at a desired velocity. These control commands are then sent to the vehicle interface to be actuated.

The RVC is in charge of keeping the vehicle within the limit of dynamics at all times, and gracefully stop the vehicle in emergency.

## API

In general, the RVC takes the commanded trajectory and various sensor readings as the inputs, which helps it build a model of the kinematic state of the vehicle and its control goals. It outputs control signals at a constant interval.

The RVC node itself does not calculate anything. At a constant interval, it iterates through all of its plugins, which are responsible for generating or modifying the control output. Refer to the RVC Plugin API section for writing your own control algorithm.

### Plugin Documentations

[Manual Override Plugin](plugin_docs/override.md)

[Compliance Plugin](plugin_docs/compliance.md)

[Input Validation Plugin](plugin_docs/input_validation.md)

[Add-Steer-Bias Plugin](plugin_docs/add_steer_bias.md)

[Gear Manager Plugin](plugin_docs/gear_manager.md)

### Publishers and Subscribers

Note that these publishers and subscribers are managed by the RVC node. RVC plugins may define their own publishers and subscribers.

| Subscription Type                           | Default Topic             | Description                                    |
|---------------------------------------------|---------------------------|------------------------------------------------|
| `race_msgs/msg/TargetTrajectoryCommand`     | `trajectory_update`       | Trajectory command from RDE                    |
| `race_msgs/msg/VehicleManualControlCommand` | `manual_command`          | Manual override command                        |
| `race_msgs/msg/VehicleKinematicState`       | `vehicle_kinematic_state` | Pose, velocity, and wheel angle of the vehicle |

| Publisher Type                        | Default Topic   | Description                                |
|---------------------------------------|-----------------|--------------------------------------------|
| `race_msgs/msg/VehicleControlCommand` | `raw_command`   | Output control command                     |
| `race_msgs/msg/RvcTelemetry`          | `rvc_telemetry` | Telemetry of the internal state of the RVC |

### Parameters

Note that these parameters are managed by the RVC node. RVC plugins may define their own parameters.

| Parameter Name                | Type             | Description                                                                 |
|-------------------------------|------------------|-----------------------------------------------------------------------------|
| `control_output_interval_sec` | `double`         | The interval for requesting and outputing control commands from the plugins |
| `max_front_wheel_angle_rad`   | `double`         | Maximum turning range of the front wheels, averaged on the two wheels.      |
| `wheelbase_m`                 | `double`         | Distance between front and rear axles                                       |
| `track_m`                     | `double`         | Wheel-to-wheel distance on the same axle                                    |
| `vehicle_weight_kg`           | `double`         | Mass of the car                                                             |
| `plugins`                     | `vector<string>` | List of plugins activated, in the order of control signal propagation       |

### RVC Plugin API

To start working with your own control algorithm and plug it in the RVC, you need to learn about how the RVC Plugin functions. More information can be found in the [ROS Iron plugin tutorial](https://docs.ros.org/en/iron/Tutorials/Pluginlib.html).

The RVC node contains a list of plugins, all of which are subclasses of `race::RvcPlugin`, defined in `plugin.hpp`. Every plugin is capable of calculating or modifying the control command. A pure pursuit controller is a plugin that fills the steering field of the output control message; a Anti-Lock Braking System (ABS) is a plugin that modifies the brake field according to wheel speed information.

**All RVC Plugin API methods should not block. If there is heavy computation associated with an API call, put it on a separate thread.**

On RVC node initialization, the plugins are loaded and initialized with the following methods called:

```cpp
/**
 * @brief Initialize plugin, saving pointers to state, config, and
 * parent node. Do not override unless needed.
 *
 * @param state Pointer to the shared state info, updated by RVC node
 * @param config Pointer to the shared config info, updated by RVC node
 * @param node Pointer to the parent node for logging, timing, etc.
 */
void RvcPlugin::initialize(
RvcState::SharedPtr state, RvcConfig::SharedPtr config,
rclcpp::Node * node);
```

```cpp
/**
 * @brief Declare ROS params, pubs, and subs, and initialize the
 * plugin here
 *
 * @return If initialization is successful
 */
bool RvcPlugin::configure();
```

Since the plugin library only allows constructor with no argument, these two methods are the workarounds.

The RVC configurations are stored in a `RvcConfig` structure managed by the RVC node and made available to the plugins by invoking `RvcPlugin::config()` or `RvcPlugin::const_config()`.

As the RVC node receives sensor inputs, a couple of callbacks are triggered on each plugin:

```cpp
void RvcPlugin::on_kinematic_update();
void RvcPlugin::on_ttl_command_update();
void RvcPlugin::on_manual_command_update();
void RvcPlugin::on_trajectory_update();
```

The inputs are stored in the `RvcState` structure managed by the RVC node and made available to the plugins by invoking `RvcPlugin::state()` or `RvcPlugin::const_state()`.

At a constant interval, the RVC node calls on each plugin to fill or modify the output control command, in the order that was configured in the RVC params. There are twp API calls:

- `bool RvcPlugin::compute_control_command(VehicleControlCommand & output_cmd)`: fills in and overwrites all or part of a fresh new control command. For example, the Pure Pursuit Controller fills in the steering field, and then the longitudinal controller fills in the throttle and brake field. A flag is returned signing success / failure.
- `bool RvcPlugin::modify_control_command(VehicleControlCommand & output_cmd)`: modifies a control command that was already filled by some plugins. For example, the ABS checks the wheel speeds and reduce the brake command when necessary.

The RVC node will call on every plugin's `compute_control_command` method before calling `modify_control_command`. For most modifier plugins (e.g. ABS), `compute_control_command` should not be overridden. For most control algorithms (e.g. Pure Pursuit), `modify_control_command` should not be overridden.

```
             pure-pursuit -> lon-control -> manual-override -> timeout -> abs  
Iteration 1     compute        compute           noop            noop     noop
Iteration 2      noop           noop            modify          modify   modify
```

## Reliability and Vulnerability

## How to Contribute a Plugin
