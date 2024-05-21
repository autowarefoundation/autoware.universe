# Planning Test Utils

## Background

Several planning components and modules have already adopted unit testing, so a common library to ease the process of writing unit tests is necessary.

## Purpose

The objective of the `planning_test_utils` is to develop a unit testing library for the planning components. This library will include

- commonly used functions
- input/mock data parser
- maps for testing
- common routes and mock data for testing.

## Available Maps

The following maps are available [here](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/planning_test_utils/test_map)

### Common

The common map contains multiple types of usable inputs, including shoulder lanes, intersections, and some regulatory elements. The common map is named `lanelet2_map.osm` in the folder.

![common](./images/common.png)

### 2 km Straight

The 2 km straight lanelet map consists of two lanes that run in the same direction. The map is named `2km_test.osm`.

![two_km](./images/2km-test.png)

The following illustrates the design of the map.

![straight_diagram](./images/2km-test.svg)

## Implemented tests

The mock data parser is unit tested to ensure there are no defects when parsing the input data.
