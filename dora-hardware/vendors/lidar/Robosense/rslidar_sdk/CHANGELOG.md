# CHANGELOG 
## Unreleased

## v1.5.10 - 2023-02-17

### Changed
- Merge RSBPV4 into RSBP


## v1.5.9 - 2023-02-17

### Changed
- Increase sending DDS buffer queue to avoid packet loss


## v1.5.8 - 2022-12-09

### Added
- Support ROS2/Humble Hawksbill
- rename RSEOS as RSE1

### Fixed
- Fix wrong widthxheight while ros_send_by_rows=true


## v1.5.7 - 2022-10-09

### Added
- Seperate RSBPV4 from RSBP
- Support to receive MSOP/DIFOP packet from rosbag v1.3.x
- Support option ros_send_by_rows

## v1.5.6 - 2022-09-01

### Added
+ Add a few build options according to rs_driver
+ Update help documents

## v1.5.5 - 2022-08-01

### Changed
- Output intensity in point cloud as float32

### Fixed
- Fix compiling and runtime error on ROS2 Elequent
- Fix frame_id in help docs

## v1.5.4 - 2022-07-01

### Added
- Support the option to stamp the point cloud with the first point

### Changed
- Remove the dependency on the protobuf library

## v1.5.3 - 2022-06-01

### Added
- Support Jumbo Mode

### Fixed
- Fix compiling error when protobuf is unavailable

## v1.5.0

### Changed
- refactory the project

### Added
- support user_layer_bytes and tail_layer_bytes
- support M2
- replace point with point cloud, as rs_driver's template parameter
- handle point cloud in rs_driver's thread

## v1.3.0 - 2020-11-10

### Added

- Add multi-cast support
- Add saved_by_rows argument
- Add different point types( XYZI & XYZIRT)

### Changed
- Update driver core, please refer to CHANGELOG in rs_driver for details
- Update some documents
- Change angle_path argument to hiding parameter

### Removed

- Remove RSAUTO for lidar type
- Remove device_ip argument

## v1.2.1 - 2020-09-04

### Fixed
- Fix bug in driver core, please refer to changelog in rs_driver for details.

## v1.2.0 - 2020-09-01

### Added
- Add camera software trigger (base on target angle)

### Changed
- Update driver core, please refer to changelog in rs_driver for details
- Update the compiler version from C++11 to C++14

## v1.1.0 - 2020-07-01

### Added
- Add ROS2 support

### Changed
- Replace while loop with cv.wait
- Update the vector copy part 
- Update the program structure

### Removed
- Remove some unused variables in message struct

## v1.0.0 - 2020-06-01

### Added
- New program structure
- Support ROS & Protobuf-UDP functions

  
