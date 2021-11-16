Signal filters design {#signal-filters-design}
=======================

# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

Some algorithms or components have an implicit assumption of having (reasonably) a smoothly varying
input signal. Another way to view this is that a component might have an undesirable response to
different frequency domains.

To avoid these undesirable responses, it is important to use a filter (in the signal processing
sense) to reject, remove, or attenuate certain frequencies within a signal.


# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

A simple interface is provided. Implementations of this interface are assumed to be stateful
and discrete-time.

The API can be seen in the
[API docs](@ref autoware::common::signal_filters::FilterBase).

The base class has some input sanitation logic so that implementations need not reimplement this
logic.

In addition, standard polymorphism is used for ease of use. This is rationalized by the idea that
these filters are intended to work with the vehicle interface on data arriving at most 100 Hz.
If these filters are needed to work on data at a higher rate, then the child classes should directly
be used or a specialized filter intended for speed should be implemented.

Filters take in a `Clock` template argument. The purpose of this is two-fold:
1. To provide support for various clocks without having to refactor (i.e. steady vs wall clock)
2. To ensure a time_point-based API is not mixed with a duration-based API

A duration-based API and a time_point-based API are provided (exclusive to one another) in order
to support different use cases a user might have.


## Assumptions / Known limits
<!-- Required -->

This API currently assumes a 1D output, and a 1D input.

If a multidimensional input or output is desired, then a different API should be provided, or the
provided API should be extended.

Implementations of this interface are assumed to be stateful and discrete-time.


## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

The API can be seen in the
[API docs](@ref autoware::common::signal_filters::FilterBase).


## Inner-workings / Algorithms
<!-- If applicable -->

Currently, this package only contains an interface definition with no logic.


## Error detection and handling
<!-- Required -->

The base class ensures the following invariants hold true:
1. Input data is not NAN or INF
2. Time step is positive

Use of SFINAE prevents the user from mixing API calls, possibly leading to the filter
being in an inconsistent state.

Care is also made to ensure modifications to the program state occurs only after exception logic to
provide strong exception guarantees.

# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

TBD by a security specialist

# References / External links
<!-- Optional -->

[Modern filters](https://en.wikipedia.org/wiki/Filter_(signal_processing)) include:

- [Butterworth filter](https://en.wikipedia.org/wiki/Butterworth_filter)
- [Chebyshev filter](https://en.wikipedia.org/wiki/Chebyshev_filter)
- [Elliptic filter](https://en.wikipedia.org/wiki/Elliptic_filter)


# Future extensions / Unimplemented parts
<!-- Optional -->

# Related issues
<!-- Required -->
- #227: Initial export
