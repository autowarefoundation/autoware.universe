BoundingBox message design {#bounding-box-design}
==========================

# Motivation

In order to ensure a modular system, it's assumed that object detectors are architecturally
delineated. In other words, software components which can produce instantaneous observations of
objects are delineated from other components in the autonomous driving stack.

This assumption or decomposition allows us to interchangeably use various forms of object detectors
with various forms of multi-object tracking (by assignment) algorithms. If a tighter coupling
between object detection and tracking is required, specialized combined stacks can be developed.

In order to facilitate the isolation of object detectors as a module, common messages types are
defined as an interface or output of the object detectors to other components.


# Use cases

The instantaneous detection of objects primarily has three use cases:

1. Collision detection (as a part of planning)
2. Tracking (which may eventually be an input to planning)
3. Region of Interest identification (as a preprocessing step to localization/mapping)

In addition, there is the implied use case that this message representation is used on a safety-
critical system.


# Requirements

## Safety-critical systems

Safety-critical systems require real-time and static memory applications. As such, this implies
that the representation of the message must be upper bounded in size.

For the purpose of performance, an optional requirement is added to message definitions such that
the maximum size is no more than `64kB`, which is also the maximum size of a single UDP packet. This
requirement ensures that latency is kept to a minimum if interprocess communication is necessary at
the interface of the object detection algorithms.


## Collision detection

In order to detect collisions, the representation must have:

1. A position at least in 2-space (with the assumption that the object is on the same plane as the
ego)
2. A bounding volume representation which fully contains the detected object
3. A way to ensure the object representation is in a consistent coordinate frame with respect to the
ego
4. Optionally uncertainty representation of the above parameters


## Tracking

The purpose of tracking is to maintain a consistent identity of a detected object over time.
In addition, tracking may also produce state estimates over time, such as kinematic estimates,
shape estimates, or label estimates.

To satisfy the fundamental requirement of tracking, the message representation must provide features
that can be used for assignment, or association of current observations with previous observations.
Features that can satisfy this requirement include:

1. Position information
2. Shape information (e.g. convex hull, bounding ellipse/box, shape features, etc.)
3. Object information (e.g. color/intensity distribution, classification label, etc.)
4. Other sensor information (e.g. velocity, heading, turn rate (via FMCW sensors))
5. Optionally uncertainty information for the above parameters

Classically, position information is the most important association feature, assuming a continuous
world and a relatively high sampling frequency.

The other features can be used to motivate various state estimates in the tracking stack.

## Region of interest detection

An object detector can also be used to identify regions of interest for various other stacks, such
as feature-based localization/mapping, or scene understanding (sign/scenario detection).

The localization/mapping use case is largely similar to the tracking use case insofar as it requires
assignment or association of features. In contrast, scene understanding requires an identification
of the relevant objects and their position in a space that is compatible with the ego frame.

As such, the features needed to satisfy this use case are similar to that of the tracking use case:

1. Position information
2. Shape/object information
3. Optionally uncertainty information


# Message definition

Based on the requirements laid out, some assumptions and simplifications can be made, and a message
type can be defined.

If any of the assumptions and simplifications must be broken for the proper operation of the
algorithm, then the choice of algorithm should be reconsidered, or a parallel construct which
encapsulates multiple software components should be made.

In the case when a stack cannot populate certain fields of the interface, the field should be in
an identifiably non-normal state, such as zero or NaN.


## Safety critical requirements

To satisfy the minimal latency requirement, the representation must be bounded in size to `64kB`.

If it's additionally assumed that `1kB` is used for secondary representation and there are
piggy-backed submessages in the communication sublayer, the maximum representation size is
`63kB`.

Next, it's assumed that some maximum number of objects that are detected instantaneously. Based on
current observations of Apex.AI urban driving use cases, typically 50-70 objects are observed per
scene. Applying a healthy safety factor gives a maximum object bound of 256 objects per frame.

These two assumptions combined lead us to have a maximum representation size of `250B`.


## Position requirements

Minimally, a 3D position is used to satisfy the position requirement inherent in all use cases. A 3D
position is used to be more general, and apply to other stacks, such as a vision/camera-based stack.

To satisfy the compatibility of representational spaces, it's assumed that all observations are made
in a common frame. This frame can then be represented by a string in the aggregate object type.
It is assumed that it is the responsibility of the consuming algorithm to have and transform the
position space into the correct coordinate frame given this information.


## Shape requirements

Next, the shape of the object is represented as a bounding box in 3 interdependent ways:

- A quaternion for orientation
- A 3D size parameter
- Four 3D positions for the corners of the bounding box

A bounding box representation was used because many objects in the autonomous driving use case
are approximately box-shaped. In addition, many efficient algorithms exist in both the 2D and 3D
case to compute bounding boxes. Finally, bounding boxes are convenient, close-formed representations
of objects, as opposed to a convex hull which can be potentially unbounded in representation size.

A quaternion is used to represent orientation. An oriented bounding box is used as opposed to
axis-aligned bounding boxes because it can better represent and fit to objects. A quaternion
is used to represent orientation in the SO(3) space because it is continuously varying without
singularities. In addition, it can be used to efficiently calculate sine and cosine values.

Finally, the size and corners of the bounding box is represented. While the two
together represent redundant information, both parameters are byproducts of the bounding box
computation process, and communicating both forms of information downstream can reduce the
computational burden and repeated work of algorithms downstream.

For example, size can be used as a coarse collision-checking step, whereas the corners are used
for fine-grained collision detection. Similarly, the size of the bounding may be treated as direct
extent observations in the tracking case, whereas the corners may be treated as proxy observations
of the centroid.


## Object/Other requirements

Of all the object features proposed, only a classification label is bounded in representation size
(assuming a finite set of classes). While the other features could be bounded, they would be
severely limited in their representational capabilities due to the size limitations.

As such, only a classification label is provided with the current form:

`vehicle_label` is an 8-bit integer depicting the classification label of the bounding box. It
can take any of the following default values:

```
NO_LABEL=0
CAR=1
PEDESTRIAN=2
CYCLIST=3
MOTORCYCLE=4
```

The `signal_label` field contains the back signal state of the car and can take any of the following
 default values:

```
NO_SIGNAL=0
LEFT_SIGNAL=1
RIGHT_SIGNAL=2
BRAKE=3
```

Next, additional kinematic fields can potentially be populated at minimal size cost. These include
velocity, heading, and heading rate, assuming a FMCW/doppler-effect sensor is available. The
inclusion of such fields can greatly improve the performance of tracking, or in some cases remove
the necessity of tracking (i.e. predictive collision detection).


## Uncertainty requirements

While uncertainty representation is not a strict requirement, the usage of uncertainty can greatly
improve the performance of downstream algorithms, both from the perspective of accuracy, and safety.

Representing the full probabilistic state of the object is difficult due to size constraints,
as representing an 8-state covariance matrix would overrun the capacity, and
similarly representing the full classification distribution is potentially unbounded in size.

As a compromise, several modeling simplifications and assumptions can be made for the bonus
representation of uncertainty.

First, it can be assumed that the uncertainty of state variables are uncorrelated, implying all
off-diagonal elements of the full covariance matrix are zero, and thus need not be represented.
Second, the coarse modeling assumption of a uniformly distributed class assignment residual can
be used. As a result, class uncertainty can be represented with a single value.


# Future extensions

The currently proposed representation for bounding boxes takes up approximately `150B` of the total
size budget of `230B`. This leaves room for up to 18 floating point values. These could be used to
cover the other optional requirements as a vector of floating points, with an optional ID denoting
the intended interpretation.


# Related issues

- #3540: Update bounding box, add use cases
- #4069: Revise new articles for the 0.12.0 release
