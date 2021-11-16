Vehicle interface design {#vehicle-interface-design}
=======================

# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

The primary external interfaces to an autonomous driving software stack are perception sensors and
the vehicle platform itself.

As such, a node which permits communication between the autonomous driving software stack and the
vehicle platform is required to control the vehicle and report on the status of the vehicle.


# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

In sum total, the vehicle interface does the following:

1. Ensure the commands sent to the vehicle are safe
2. Send commands to the vehicle
3. Receive information from the vehicle
4. Support multiple vehicle platforms/communication mechanisms

To support these use cases, the following architecture was devised:

![Vehicle interface architecture](images/VehicleInterfaceUML.svg)

The following components in the architecture are a direct concern of the vehicle interface package:

1. Safety state machine
2. Platform interface
3. Vehicle interface node


## Safety state machine

The safety state machine ensures commands sent to the vehicle are safe and consistent.

All methods intended to be used during runtime are marked `noexcept` in that nothing here is
expected to ever fail.

As a result, warnings are reported via a separate method (rather than by exceptions). The rationale
is that the state machine should always be able to make commands safe, implying commands are always
generated. By contrast, throwing an exception would break this path (of generating commands),
necessitating an alternate path to report warnings.

### Assumptions / Known limits
<!-- Required -->

This component has no assumptions past what is enforced by the API. It is the job of this component
to enforce assumptions.


### Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

The proposed API can be [seen here](@ref autoware::drivers::vehicle_interface::SafetyStateMachine).


### Inner-workings / Algorithms
<!-- If applicable -->

The following behaviors are expected out of the safety state machine:

1. Any gear shifts between Park, Reverse, and Drive should be removed if the vehicle has non-zero
velocity
2. Headlights should be active if wipers are active
3. Commands should be clamped within a pre-specified range
4. If there is large error between a raw and clamped value, then a warning should be raised
5. Validating that high frequency components are small in the control command should happen here;
if high frequency components are present, a warning should be raised
6. A warning should be raised if a state command does not result in a change in reported state
within a certain time period


### Error detection and handling
<!-- Required -->

On problematic cases (points 4, 5 and 6 in the previous section), warnings or exceptions should be
raised.

Whether a warning or exception is used, should be parametrizable.


## Platform interface

The platform interface permits sending and receiving data from the vehicle platform.


### Assumptions / Known limits
<!-- Required -->

The platform interface is assumed to encapsulate the communication mechanism with the vehicle,
and any libraries or utilities needed to translate platform-specific messages or commands.

Any code generation based on DBC files is considered to be outside of the scope of this design.
Such auto-generated code could be used as a library within a platform interface implementation.

Any failure to communicate with the vehicle platform should be reported as it is considered a
critical error.


### Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

The proposed API can be [seen here](@ref autoware::drivers::vehicle_interface::PlatformInterface).

The virtual interface uses return codes because it is not possible to obligate throwing an exception
on an error, whereas missing a return statement is usually a compilation error.


### Inner-workings / Algorithms
<!-- If applicable -->

This is primarily an interface. There is some logic to prevent the sending of commands when the
vehicle is not in autonomous mode.


### Error detection and handling
<!-- Required -->

The primary failure mode of the platform interface is when sending or receiving data to or from the
vehicle platform fails.

While the implementer may include some retry mechanisms in their implementation of the platform
interface, a failure should ultimately be reported to the `VehicleInterfaceNode` via a return
code or a thrown exception.

In these cases, communication failure with the interface is treated as a critical error and should
result in some safety actions being taken.


## Vehicle interface node

The vehicle interface node wraps all components needed for communication with the vehicle.


### Assumptions / Known limits
<!-- Required -->

It is generally assumed that instantiations of the vehicle interface node will be implemented as
child classes in separate packages.

This pattern would be preferred over a factory method to improve extensibility and ensure that
"you get what you pay for", meaning a developer will not have to have dependencies on other
interface implementations to use a particular interface.


### Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

The proposed API can be
[seen here](@ref autoware::drivers::vehicle_interface::VehicleInterfaceNode).

As a node, the inputs are:

1. One of the following:
  - `VehicleControlCommand` (deprecated in favor of `AckermanControlCommand`)
  - `AckermannControlCommand`
  - `RawControlCommand`
  - `HighLevelControlCommand`
2. `VehicleStateCommand`

Additionally, each vehicle interface or specific vehile may support features which other vehicle
interfaces or specific vehicles do not. To accommodate this, a list of features of type
[autoware::drivers::vehicle_interface::ViFeature](@ref autoware::drivers::vehicle_interface::ViFeature)
must be passed to the [VehicleInterfaceNode](@ref autoware::drivers::vehicle_interface::VehicleInterfaceNode)
class. Adding a feature to this list indicates that the drive-by-wire or simulator interface
supports this feature but doing so does not automatically enable the feature. To enable a feature,
it must also be added to a `features` parameter when the node is configured. Adding a feature
to the `features` parameter indicates that it is supported by the specific vehicle being used.
A feature must be added to both the VehicleInterfaceNode child class and the `features` parameter
for the publisher and subscriber for that feature to be enabled. See the [ViFeature](@ref autoware::drivers::vehicle_interface::ViFeature)
enum for available features.

The outputs are then:

1. `VehicleOdometry`
2. `VehicleStateReport`
3. Additional sensor messages based on the vehicle platform


### Inner-workings / Algorithms
<!-- If applicable -->

The vehicle interface node itself has relatively little logic. It primarily offloads logic
to the other components in this document and in the architecture.


### Error detection and handling
<!-- Required -->

A variety of failure modalities are handled. These are largely encapsulated by
the safety state machine and the (optional) use of a low pass filter on the control
commands.

Two additional error handling mechanisms are implemented on the node level:

1. Old data is ignored
2. The vehicle interface will come to a smooth stop with hazard lights on in the event of
no commands from the ADAS stack

For more details on various error conditions and mitigation strategies, see the
[Vehicle interface failure analysis](@ref vehicle-interface-failure-analysis).


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

The platform interface directly interacts with the vehicle platform. This is an external
communication channel that may not use DDS.

Additional security mechanisms may be needed here.


# References / External links
<!-- Optional -->

- [Vehicle interface prior art](@ref vehicle-interface-prior-art)
- [Vehicle interface high-level design](@ref vehicle-interface-high-level-design)
- [Vehicle interface failure analysis](@ref vehicle-interface-failure-analysis)


# Future extensions / Unimplemented parts
<!-- Optional -->

The following has not yet been implemented:

1. Velocity control

# Related issues
<!-- Required -->
- #230: Initial export
