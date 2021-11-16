Vehicle interface high-level design {#vehicle-interface-high-level-design}
==========================================================================

@tableofcontents

A vehicle interface interfaces a general autonomous driving software stack with a specific vehicle.
This includes the transmission of commands *to* the vehicle hardware, and the receipt of reports or
sensor data *from* the vehicle platform.

Until a standard interface is defined, each different type of vehicle will need slightly different
vehicle interfaces.


# Use cases

This section defines general use cases and settings a generic vehicle interface might encounter.

A vehicle interface should broadly do the following:

1. Act as a replacement for a human in the driver seat
2. Provide low-level access to hardware
3. Prevent unsafe actions, combinations, or sequences of actions
4. Facilitate the easy development of autonomous driving software stacks
5. Allow a physical driver to take control preferentially


## Replace the physical presence of a human driver

For the purpose of actuating the vehicle and receiving the information needed to appropriately
actuate the vehicle safely, the vehicle interface should replace a human driver.

Other aspects such as prediction, planning, routing are out of scope.

Actuation generally refers to controlling the steering, throttle, braking, gears, and indicators
of the vehicle.

Receiving information generally refers to information found on the driver's dashboard, such as
vehicle velocity, fuel information, and the current state of indicators, gears, and so on.


## Provide low-level hardware access

Certain vehicle platforms may have a more granular set of sensors that are not directly exposed to
a human driver, such as individual wheel odometry, or front facing radars. The signals from these
sensors should additionally be exposed to the wider autonomous driving system stack.


## Prevent unsafe (sequences of) actions

The vehicle interface should prevent actions that are known to be unsafe or damaging to a vehicle.

This can include rapid actuation of the brake, throttle, or steering, and shifting gears to park
when the vehicle has non-negligible velocity.


## Facilitate ADAS development

The vehicle interface should provide interfaces with varying levels of granularity. A low
granularity interface should be provided to abstract a significant amount of control and edge cases
from the user. A high granularity interface should be provided to expose the full functionality of
the hardware-software interface to the software stack.

While "easy development" is ill-defined, an approximation would be to minimize the number of degrees
of input freedom.

More concretely, development can be facilitated by supporting the following use cases:

1. Using a game pad to control the vehicle platform (to minimally test the vehicle interface and the
drive-by-wire interface)
2. Controlling the vehicle with a combined longitudinal and lateral controller/planner that is aware
of some vehicle details, without having to directly control gear shifting
3. Using a geometric path-planner with a decoupled velocity (longitudinal) control, with little
knowledge of the vehicle platform

In addition, depending on the amount of logic present in the vehicle interface, it may be useful for
the interface to be compatible with simulators, so that the interface logic can be appropriately
tested and exercised.

Finally, making it possible to support multiple hardware-software communication mechanisms would
also simplify development.


## Allow a physical driver to take preferential control

If the vehicle interface is the only arbiter of control to the vehicle platform, then the vehicle
interface should preferentially permit physical actuation of the vehicle. In the case of
development, this should always be the case. In a production vehicle, it may be appropriate to never
permit physical intervention.

Many vehicle platforms may allow this implicitly by how the drive-by-wire interface operates.


# Requirements

The use cases above can be decomposed into requirements:

1. Replacing a physical driver
  1. Provide a means to directly control the following aspects of the vehicle:
    1. Throttle
    2. Braking
    3. Steering
    4. Gear state
    5. Headlights
    6. Turn signals
    7. Hazard lights
    8. Horn
    9. Wipers
  2. Provide a means to report the following information from the vehicle platform:
    1. Velocity
    2. Steering angle
    3. Gear state
    4. Headlight state
    5. Turn signal state
    6. Hazard light state
    7. Horn
    8. Wiper state
  3. Control of the vehicle is assumed to be with a zero-order hold between commands
2. Provide low-level hardware access
  1. Report additional sensor information, examples of which can include:
      - RADAR
      - IMU
      - GPS
      - Ultrasonics
  2. Provide an additional API for additional control (e.g. individual wheel torques)
3. Prevent unsafe actions
  1. Prevent high frequency oscillation in control signals
  2. Prevent shifts to parking gear with non-zero velocity
  3. Prevent activation of ADAS functionality when the vehicle cabin and contents are not secured
  4. Ensure headlights are active while wipers are running (California law)
  5. This functionality should be optional as it runs counter to providing low-level access to the
  vehicle platform
4. Facilitate ADAS development
  1. Provide a high-level interface to control velocity
  2. Provide a high-level interface to control path curvature
  3. Permit different communication mechanisms
    1. Connect with a simulator
5. Allow a physical driver to take preferential control
  1. The software should block all control commands to the vehicle platform if manual control is
detected


# Mechanisms

To satisfy the requirements of the vehicle interface, the following components (which roughly
correspond to classes in object-oriented programming) are defined:

1. Control messages
  1. `RawControlCommand`
  2. `VehicleControlCommand`
  3. `HighLevelControlCommand`
2. Other interface messages
  1. `VehicleStateCommand`
  2. `VehicleOdometry`
  3. `VehicleStateReport`
3. Vehicle platform hardware communication mechanism
4. Vehicle-specific interface
5. Safety state machine
6. Vehicle interface
7. Low pass filter
8. Controller

This components are described in detail further below. All components can be combined in a manner
specified by the below UML diagram:

![Vehicle Interface Architecture](images/VehicleInterfaceUML.svg)


## Control Messages

A fundamental requirement of the vehicle interface is to manipulate the kinematic state of the
vehicle. As such, an interface which defines the requested kinematic state is required.

Multiple message types are defined for vehicle control. This is because no one message type can
appropriately cover the desired use cases (of easy development, preventing unsafe actions), while
remaining as concrete and unambiguous as possible.

For each message type, both longitudinal and lateral control actions are combined into a single
type, and predefined message definitions are not used. The purpose of the former is to remove the
need for time synchronization or message alignment within the vehicle interface. The latter is
imposed in order to minimize the size of the communication messages and remove ambiguity.

In all cases, these messages are intended to be published at a high frequency, approximately 100 Hz.


### RawControlCommand

This message type is defined as follows:

```
builtin_interfaces/Time stamp
uint32 throttle
uint32 brake
front_steer int32
rear_steer int32
```

This type is intended to be used when direct control of the vehicle hardware is required. As such,
no assumptions about units are made. It is the responsibility of the system integrator to ensure
that the values used are appropriate for the vehicle platform.


### VehicleControlCommand

This message type is defined as follows:

```
builtin_interfaces/Time stamp
# should be negative when reversed
float32 long_accel_mps2 0.0
float32 front_wheel_angle_rad 0.0
float32 rear_wheel_angle_rad 0.0
```

The message semantics represent instantaneous control of the vehicle (i.e. hands on steering wheel
and feet on pedals). This type is intended to be used as a middle ground for representational power,
exposing direct wheel angles, but abstracting away some braking/throttling details.

A vehicle interface using this type as the primary input is expected to include at a minimum the
safety state machine.

The rear wheel angle is kept for possible future extensions to Quadrasteer or rear-steering
platforms.


### HighLevelControlCommand

This message type is defined as follows:

```
builtin_interfaces/Time stamp
# should be negative when reversed
float32 velocity_mps 0.0
float32 curvature
```

This type is intended to be used as an interface when only simple control is required, and thus
abstracts away more direct control of the vehicle.

A vehicle interface using this type as the primary input is expected to include at a minimum
the safety state machine and the controller.


## Other interface messages

Additional interfaces types are defined for the vehicle interface.


### VehicleStateCommand

This message type is defined as follows:

```
# VehicleStateCommand.msg

builtin_interfaces/Time stamp
uint8 blinker 0
uint8 headlight 0
uint8 wiper 0
uint8 gear 0
uint8 mode 0
bool hand_brake false
bool horn false
bool autonomous false

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
uint8 GEAR_REVERSE = 1
uint8 GEAR_PARK = 2
uint8 GEAR_LOW = 3
uint8 GEAR_NEUTRAL = 4
```

This message represents lower frequency control of all other aspects of the vehicle related to
driving.

This message is intended to be published at some lower, unspecified rate, such as 1-10 Hz.


### VehicleOdometry

This message type is defined as follows:

```
# VehicleOdometry.msg
builtin_interfaces/Time stamp
float32 velocity_mps 0.0
float32 front_wheel_angle_rad 0.0
float32 rear_wheel_angle_rad 0.0
```

This message represents common sensing information from the vehicle platform needed to estimate
the vehicle's kinematic state.

This data is expected to be high frequency, and can be treated as "best effort".


### VehicleStateReport

This message type is defined as follows:

```
# VehicleStateReport.msg

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
uint8 GEAR_REVERSE = 1
uint8 GEAR_PARK = 2
uint8 GEAR_LOW = 3
uint8 GEAR_NEUTRAL = 4
# Autonomous
uint8 MODE_MANUAL = 0
uint8 MODE_NOT_READY = 1
uint8 MODE_AUTONOMOUS = 2
uint8 MODE_DISENGAGED = 3
```

This message represents the non-control related state of the vehicle platform.

It is intended to be used to update state machines in the vehicle interface and behavior planners.

This data is expected to be moderate in frequency, and can be treated as "best effort".


## Vehicle platform hardware communication mechanism

Each vehicle platform has some communication mechanism by which commands can be transmitted, and
reports received.

Most vehicle platforms use CAN. Supporting simulators may necessitate the use of custom interfaces,
or other communication mechanisms, such as ROS 1/2 publishers, or websockets.


## Vehicle-specific interface

Each vehicle platform may have different control characteristics, for example a vehicle-specific
brake or throttle table.

The purpose of this component is to convert general, platform agnostic commands into
vehicle-specific data, and to transmit these commands to the vehicle platform itself.

Similarly, this component must receive reports from the platform, and translate this information
to a vehicle-agnostic message.

As the communication mechanism may vary from platform to platform, this component should encapsulate
whichever communication mechanism the platform makes use of.


## Safety state machine

The safety state machine maintains an internal state that minimally represents the vehicle platform.
This state machine is then used to translate a potentially incompatible set of commands to a
consistent set of commands.

Concretely, the state of this safety state machine should be approximately what is represented in
the `VehicleStateReport` type (it may be a subset or superset).

An example of some of the combinations of commands that this state machine should reject are
requirements 3.2, 3.3, and 3.4.


## Vehicle interface

The vehicle interface is a higher level component which provides an interaction point with the
larger ADAS stack via ROS 2's IPC mechanisms. All other components listed here are composed into
this component.


## Low pass filter

A generic low-pass filter, or any equivalent filter (e.g. Butterworth, Chebyshev, etc.) should be
optionally available to remove high frequency noise from the control signals.


## Controller

A controller should be optionally available in case the developer wishes to control the vehicle via
velocity rather than acceleration. This can be any simple reference-tracking controller, such as a
PID controller.


# Related issues
- #230: Initial export
