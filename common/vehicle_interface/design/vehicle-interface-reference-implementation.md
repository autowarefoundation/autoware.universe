Vehicle-Interface-Reference-Implementation {#vehicle-interface-reference-implementation}
==========================================

@tableofcontents

This document defines a reference implementation for the vehicle interface component.
A software component (i.e. a library, executable, set of executables, or software black box) that
satisfies at least the inputs, outputs, and behaviors defined in this document shall be able to
operate with any sufficiently compliant Autoware.Auto-derived autonomous driving stack.

# Introduction

A vehicle interface is the software component at the lowest point in the autonomous driving stack.
It translates the commands from the stack into actuations on the physical vehicle. It is the
responsibility of this component to act as a translator from software to hardware.

# Scope

This document lays out a minimal set of inputs, outputs and behaviors for a vehicle interface.

In particular, this document defines message types for input and output, and the rationale behind
their definition. The behaviors are defined in terms of expected results, and the rationale is
similarly outlined.

An implementation compliant with these inputs, outputs and behaviors will be able to operate with
any sufficiently compliant Autoware.Auto-derived autonomous driving stack.

Additional behaviors, and outputs can be used in a vehicle interface implementation. These should
be properly documented. If additional inputs are used, then the minimal behavior outlined in this
document should still be possible without the inclusion of the extra inputs.

# Reference Implementation

This section outlines the inputs, outputs and behaviors required for a minimal vehicle interface
implementation

## Inputs

A vehicle interface is expected to take two inputs:

- Vehicle Control Command Message: Actuation commands to control a vehicle's trajectory
- Vehicle State Command Message: Secondary commands for the rest of the vehicle's state

### Vehicle Control Command Message

A vehicle control command message has the following form:

```
builtin_interfaces/Time stamp
float32 long_accel_mps2
float32 front_wheel_angle_rad
float32 rear_wheel_angle_rad
```

This message type is intended to provide a proxy for a foot on the accelerator and brake pedal,
and a pair of hands on the steering wheel, while also providing a convenient representation
for controller developers.

These wheel angles are counter-clockwise positive, where 0 corresponds to a neutral, or
forward-facing orientation.

**Default Values**: The default value for all fields is 0

**Rationale**: A time stamp field is provided because a control command should be generated in
response to the input of a vehicle kinematic state. Retaining the time of the triggering data
should aid with diagnostics and traceability.

**Rationale**: A single acceleration field is provided because controllers typically plan a
longitudinal acceleration and turning angle for the rigid body. Braking is baked into this single
command for simplicity of controller development, and because it is generally not expected for both
the brake to be depressed and the accelerator at the same time.

**Rationale**: Wheel steering angles are provided since it allows the controller to only require a
simplified view of the vehicle platform, such as knowing the wheelbase length. Depending on the
implementation of a drive-by-wire interface, producing a given wheel angle may require knowledge
of the mapping between steering wheel angle to wheel angle.

**Rationale**: The rear wheel angle is also provided to enable support for rear drive vehicles,
such as forklifts.

**Rational**: A frame is not provided since it is implied that it is fixed to the vehicle frame.

For more details on the message, see the formal message definition in the package
`autoware_auto_msgs`. While this message and it's rationales are duplicated here for convenience,
the message package should be taken as the source of truth.

### Vehicle State Command Message

The vehicle state command message has the following form:

```
builtin_interfaces/Time stamp
uint8 blinker
uint8 headlight
uint8 wiper
uint8 gear
uint8 mode
bool hand_brake
bool horn
bool autonomous

### Definitions
# Blinker
uint8 BLINKER_OFF = 0
uint8 BLINKER_LEFT = 1
uint8 BLINKER_RIGHT = 2
uint8 BLINKER_HAZARD = 3
# Headlight
uint8 HEADLIGHT_OFF = 0
uint8 HEADLIGHT_ON = 1
uint8 HEADLIGHT_HIGH = 2
# Wiper
uint8 WIPER_OFF = 0
uint8 WIPER_LOW = 1
uint8 WIPER_HIGH = 2
uint8 WIPER_CLEAN = 3
# Gear
uint8 GEAR_DRIVE = 0
uint8 GEAR_REVERSE = 0
uint8 GEAR_PARK = 2
uint8 GEAR_LOW = 3
uint8 GEAR_NEUTRAL = 4
```

This message is intended to control the remainder of the vehicle state, e.g. those not required for
minimal collision-free driving.

**Default Values**: The default value for all fields should be zero.

**Rationale**: Hazard lights superseded a left/right signal, and as such are an exclusive state

**Rationale**: Cleaning might be necessary to ensure adequate operation of sensors mounted behind
the wind shield

**Rationale**: A horn might be required to signal to other drivers

**Rationale**: Autonomous is a flag because this is a command message: true requests a transition
to autonomous, false requests a disengagement to manual.

**Rationale**: While additional states are possible for other fields (e.g. headlights, wipers,
gears, etc.), this message only prescribes the minimal set of states that most or all vehicles
can satisfy.

For more details on the message, see the formal message definition in the package
`autoware_auto_msgs`. While this message and it's rationales are duplicated here for convenience,
the message package should be taken as the source of truth.


## Outputs

The vehicle interface is expected to produce two outputs:

- Vehicle Odometry message: reports kinematic information
- Vehicle state report message: reports information about the vehicle's non-kinematic state

If the vehicle has additional information available, such as via a built-in RADAR, GPS,
or IMU, then this information should be output on a separate topic of appropriate type.

### Vehicle Odometry Message

The vehicle odometry message has the following form:

```
builtin_interfaces/Time stamp
float32 velocity_mps
float32 front_wheel_angle_rad
float32 rear_wheel_angle_rad
```

This message reports the kinematic state of the vehicle as the vehicle itself reports. The intended
use case for this message could be in motion planning for initial conditions, or dead reckoning.

**Default Values**: The default value for this message should be 0 in all fields.

**Rationale**: This message is separate from the vehicle state report since they are typically for
distinct use cases (e.g. behavior planning vs dead reckoning)

**Rationale**: A vehicle is expected to have encoders which provide velocity, and steering angle.
Acceleration and other fields may not be available on all vehicle platforms.

**Rationale**: A stamp is expected here because this is timely information. In this sense, the
vehicle interface is acting as a sensor driver.

**Rationale**: A frame id is not provided because these are assumed to be fixed to the vehicle
frame, and heading information is not available.

For more details on the message, see the formal message definition in the package
`autoware_auto_msgs`. While this message and it's rationales are duplicated here for convenience,
the message package should be taken as the source of truth.

### Vehicle State Report Message

The vehicle state report message has the following form:

```
builtin_interfaces/Time stamp
uint8 fuel # 0 to 100
uint8 blinker
uint8 headlight
uint8 wiper
uint8 gear
uint8 mode
bool hand_brake
bool horn

### Definitions
# Blinker
uint8 BLINKER_OFF = 0
uint8 BLINKER_LEFT = 1
uint8 BLINKER_RIGHT = 2
uint8 BLINKER_HAZARD = 3
# Headlight
uint8 HEADLIGHT_OFF = 0
uint8 HEADLIGHT_ON = 1
uint8 HEADLIGHT_HIGH = 2
# Wiper
uint8 WIPER_OFF = 0
uint8 WIPER_LOW = 1
uint8 WIPER_HIGH = 2
uint8 WIPER_CLEAN = 3
# Gear
uint8 GEAR_DRIVE = 0
uint8 GEAR_REVERSE = 0
uint8 GEAR_PARK = 2
uint8 GEAR_LOW = 3
uint8 GEAR_NEUTRAL = 4
# Autonomous
uint8 MODE_MANUAL = 0
uint8 MODE_NOT_READY = 1
uint8 MODE_AUTONOMOUS = 2
uint8 MODE_DISENGAGED = 3
```

**Default Value**: N/A. All fields should be populatable by the vehicle interface.

**Rationale**: A discrete fuel range is provided as finer granularity is likely unnecessary.

**Rationale**: A simple state machine is provided to ensure that disambiguate between intentionally
manual, and two modes of unintentionally being in manual mode: due to preconditions not being met,
or due to some failure of the autonomous driving stack.

For more details on the message, see the formal message definition in the package
`autoware_auto_msgs`. While this message and it's rationales are duplicated here for convenience,
the message package should be taken as the source of truth.

## Behaviors

At least the following behaviors are expected from a compliant vehicle interface implementation:

1. Command mapping: Maps control commands to logical inputs to the vehicle
2. Zero order hold: Commands a constant actuation command in between receipt of command messages
3. State reporting: Reports the current state of the vehicle at regular intervals
3. Unsafe command rejection: Rejects unsafe commands or command sequences
4. Timeout fallback behavior: Safely come to a stop if no commands are received
5. Manual override: Physical actuation of the controls should result in a state transition and
always be preferred over driver input


### Command mapping

The vehicle interface must map a vehicle control command message to actuations in the physical
vehicle that result in the requested commands.

This may involve some mapping between current velocity, and requested information to a particular
throttle voltage.

Similarly for a steering angle, a voltage may be commanded to satisfy a given wheel angle, according
to some mapping from steering wheel angle to front wheel angle.

**Rationale**: Using commands in this form simplifies controller development, and isolates vehicle
specific information to the vehicle interface.

If a nonzero value is provided in a field, that the interface cannot satisfy (e.g. rear wheel
angle), then that field shall be ignored and a warning shall be reported.

**Rationale**: It is outside the vehicle interface's concerns to attempt to correct vehicle dynamics
a nonzero value when not expected corresponds to a warning because the stack is either not
configured correctly, or data is incorrectly initialized.

The vehicle interface must also satisfy the following mapping:

| Current Velocity \ Commanded Acceleration | +                          | -                            | 0     |
|-------------------------------------------|----------------------------|------------------------------|-------|
| +                                         | Throttle                   | Brake                        | Brake |
| -                                         | Brake                      | Throttle                     | Brake |
| 0                                         | (Shift to Drive), Throttle | (Shift to Reverse), Throttle | Brake |

**Rationale**: This is intended to simplify controller development. This allows the controller
to keep signs consistent for both forward and reverse. In addition, this removes the need to command
gear shifting behaviors for complicated forward-reverse maneuvers (e.g. parallel parking),
simplifying motion planner development.

### Zero order hold

Between control commands, the last commanded throttle, steer angle(s), and vehicle states should be
maintained.

**Rationale**: The vehicle interface does not have future planning information (e.g. a trajectory),
so it cannot and should not attempt to do any extrapolation. Explicitly holding the last command
provides consistent and repeatable behavior to simplify controller development.

### State Reporting

The vehicle interface should report the vehicle's state and odometry on the appropriate topics at
regular rates.

**Rationale**: This information is needed by the rest of the autonomous driving stack, e.g. for
behavior planning purposes, state estimation, and so on.

### Unsafe Command Rejection

Unsafe combinations of commands should be ignored or smoothed.

These unsafe behaviors include, but are not limited to:
- Shifting between drive, park, and reverse with a nonzero velocity
- High frequency oscillations in control commands

**Rationale**: Gear shifting behavior should be limited when the vehicle velocity is nonzero.
If gear shifting were allowed, the car may be damaged, and user discomfort or injury may result.
Gear shifting between drive, park and reverse with nonzero velocity is never a valid maneuver.

**Rationale**: High frequency oscillations in control commands indicate a problem upward in the
stack. This behavior can result in user discomfort, or a fault in the vehicle's electrical control
system being triggered.

### Timeout Fallback Behavior

If a control command has not arrived within a specified period, the vehicle interface should
revert to a simple fallback behavior.

This fallback behavior should include:
- Activating the hazard lights
- Keeping the steering angles straight
- Coming to a stop at an assertive rate

**Rationale**: If a fault in the upstream stack occurs, some simple default behavior should
be available that can bring the vehicle to a safe state.

**Rationale**: Hazard lights are a standard means to signal to other drivers that a fault has
occurred.

**Rationale**: A straight trajectory is most predictable for other drivers, and less likely
to veer into a non-drivable space

**Rationale**: Assertive deceleration should occur such that the vehicle can quickly come to
a stop, but not so aggressively that other drivers do not have time to react.

### Manual Override

When the steering wheel, brake, or throttle of the vehicle is manipulated, the vehicle interface
should disengage and cede control to the user.

**Rationale**: Autonomous vehicles are still in development. A safety driver taking control
likely knows the preferred course of action.

When the vehicle state is manually actuated, this command should be preferred to the autonomous
driving stack's commands (e.g. headlights, hazard lights, horn, etc.)

**Rationale**: Autonomous vehicles are still in development. A safety driver taking control
likely knows the preferred course of action. In addition, fully overriding all modes of inputs
is possibly difficult for a vehicle interface provider.

# References

- AutowareAuto#62
- [Autoware.AI Vehicle Interface Design](https://github.com/CPFL/Autoware/issues/1541)
- [DataSpeed ADAS Kit](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/downloads/ADAS_Kit_20180606.zip)

## Related Message Types

Apollo:
- [VehicleSignal](https://github.com/ApolloAuto/apollo/blob/fb12723bd6dcba88ecccb6123ea850da1e050171/modules/common/proto/vehicle_signal.proto)
- [Lexus](https://github.com/ApolloAuto/apollo/blob/51651b9105e55c14e65cc2bd349b479e55fffa36/modules/canbus/proto/lexus.proto)
- [DriveState](https://github.com/ApolloAuto/apollo/blob/51651b9105e55c14e65cc2bd349b479e55fffa36/modules/common/proto/drive_state.proto)
- [VehicleState](https://github.com/ApolloAuto/apollo/blob/51651b9105e55c14e65cc2bd349b479e55fffa36/modules/common/vehicle_state/proto/vehicle_state.proto)

Autoware:
- [AccelCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/AccelCmd.msg)
- [BrakeCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/BrakeCmd.msg)
- [ControlCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/ControlCommand.msg)
- [IndicatorCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/IndicatorCmd.msg)
- [LampCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/LampCmd.msg)
- [RemoteCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/RemoteCmd.msg)
- [SteerCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/SteerCmd.msg)
- [VehicleCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/VehicleCmd.msg)
- [VehicleStatus](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/VehicleStatus.msg)
