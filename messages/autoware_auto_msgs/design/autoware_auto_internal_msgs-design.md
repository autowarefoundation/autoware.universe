autoware_auto_msgs internal message design
=========================

[TOC]

This document is intended to track the design and rationale of messages internal to individual
components/stacks.

# Perception

This supersection tracks messages relating to the perception components of the autonomous driving
stack.

## Object Detection

This section tracks internal messages needed for the implementation of object detection stacks
that result in the emission of the `BoundingBoxArray` type as a common object list representation.

### Classical 3d object detection

This section covers the internal messages needed by a classical 3d object detection stack,
consisting of ground filtering, downsampling, clustering, and finally hull formation.

#### PointClusters

```
sensor_msgs/PointCloud2[] clusters
```

This message represents a set of point clusters as a result of object detection or clustering.
`PointCloud2` was used as a cluster can be conceived as a subset of a point cloud, and
`PointCloud2` is the standard representation for point clouds.


## Tracking

This section tracks messages internal to specific implementation of tracking stacks.

# Localization

This supersection tracks messages relating to the implementation of localization/mapping
components in the autonomous driving stack.

# Planning

This supersection tracks messages relating to the implementation of planning components in the
autonomous driving stack.

## Global Planning

This section tracks internal messages needed for the implementation of specific global planning
stacks.

## Behavior Planning

This section tracks internal messages needed for the implementation of specific behavior planning
stacks.

## Motion Planning

This section tracks internal messages needed for the implementation of specific motion planning
stacks.

## Control

This section tracks internal messages needed for the implementation of specific controller
stacks.
