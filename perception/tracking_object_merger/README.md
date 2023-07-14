# Tracking Object Merger

## Purpose

This package try to merge two tracking objects from different sensor.

## Inner-workings / Algorithms

Merging tracking objects from different sensor is a combination of data association and state fusion algorithms.

Detailed process depends on the merger policy.

### decorative_tracker_merger

In decorative_tracker_merger, we assume there are dominant tracking objects and sub tracking objects.
Sub tracking objects are merged into dominant tracking objects.

### equivalent_tracker_merger

This is future work.
