# Architecture Overview

## Introduction

Currently it is difficult to improve Autoware.AI's capabilities due to a lack of concrete architecture design and a lot of technical debt, such as the tight coupling between modules as well as unclear responsibilities for each module. At Tier IV, we thought that a new architecture was needed to help accelerate the development of Autoware.

The purpose of this proposal is to define a layered architecture that clarifies each module's role and simplifies the interface between them. By doing so:

- Autoware's internal processing becomes more transparent
- Collaborative development is made easier because of the reduced interdependency between modules
- Users can easily replace an existing module (e.g. localization) with their own software component by simply wrapping their software to fit in with Autoware's interface

Note that the initial focus of this architecture design was solely on driving capability, and so the following features were left as future work:

- Fail safe
- HMI
- Real-time processing
- Redundant system
- State monitoring system

## Use Cases

When designing the architecture, the use case of last-mile travel was chosen. For example:

**Description:** Travelling to/from a grocery store in the same city  
**Actors:** User, Vehicle with Autoware installed (hence referred to as "Autoware")  
**Assumptions:**

- Environment is an urban or suburban area less than 1 km^2.
- Weather conditions are fine
- Accurate HD map of the environment is available

**Basic Flow:**

1. **User:** Starts a browser on their phone and accesses the Autoware web app. Presses "Summon", and the app sends the user’s GPS location to Autoware
2. **Autoware:** Plans a route to the user’s location, and shows it on the user’s phone
3. **User:** Confirms the route and presses “Engage”
4. **Autoware:** Starts driving autonomously to the requested location and pulls over to the side of the road on arrival
5. **User:** Gets in the vehicle and presses "Go Home"
6. **Autoware:** Plans the route to the user’s location
7. **User:** Confirms the route and presses “Engage”
8. **Autoware:** Drives autonomously to the user's home

## Requirements

To achieve this last-mile use case, the following functional requirements for Autoware were set:

- Autoware can plan a route to the specified goal within the type of environment described above.
- Autoware can drive along the planned route without violating any traffic rules.
- (Nice to have) Autoware provides a comfortable ride with smooth acceleration and limited jerk.

Since Autoware is open source and is meant to be used/developed by people around the world, we also set some non-functional requirements for the architecture:

- Architecture can be extended for use with new algorithms without having to change the interface
- Architecture can be extended to follow traffic rules for different countries
- The role and interface of a module must be clearly defined

## High-level Architecture Design

![Overview](image/Overview2.svg)

This new architecture consists of the following six stacks. Each of these design pages contains a more detailed set of requirements and use cases specific to that stack:

- [Sensing](Sensing/Sensing.md)
- [Localization](Localization/Localization.md)
- [Perception](Perception/Perception.md)
- [Planning](Planning/Planning.md)
- [Control](Control/Control.md)
- [Vehicle](Vehicle/Vehicle.md)
- [Map](Map/Map.md)

## References

- [New architecture presentation given to the AWF Technical Steering Committee, March 2020](https://discourse.ros.org/uploads/short-url/woUU7TGLPXFCTJLtht11rJ0SqCL.pdf)
