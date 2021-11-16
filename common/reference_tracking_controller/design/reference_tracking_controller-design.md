Reference tracking controller design {#reference-tracking-controller-design}
=======================

# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This package is intended to provide a common API and some common implementations of simple
controllers, for example PID controllers. The purpose of these controllers are to provide simple
feedback control of 1-dimensional signals.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

A simple interface is provided. Implementations may be stateful or stateless. If a multidimensional
input or output is desired, then a different API should be provided, or the provided API should be
extended.

The API can be seen in the
[API docs](@ref autoware::common::reference_tracking_controller::ReferenceTrackerBase).


## Assumptions / Known limits
<!-- Required -->

This API currently assumes a 1D output, and a 1D input, potentially with a single derivative.

When derivatives are not provided, then they are assumed to be zero (i.e. zero-order hold).

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

The API can be seen in the
[API docs](@ref autoware::common::reference_tracking_controller::ReferenceTrackerBase).

## Inner-workings / Algorithms
<!-- If applicable -->

Currently, this package only contains an interface definition with no logic.

## Error detection and handling
<!-- Required -->

Currently, this package only contains an interface definition with no logic.

# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->


# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->

- A PID controller will be implemented, #4999

# Related issues
<!-- Required -->
- #230: Initial export
