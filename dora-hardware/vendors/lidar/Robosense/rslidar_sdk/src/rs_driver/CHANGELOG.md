# CHANGLOG

## Unreleased

### Changed 


## v1.5.10 2023-04-11

### Added
- Merge RSBPV4 into RSBP


## v1.5.9 2023-02-17

### Added
- Support CRC32 check on MSOP/DIFOP packets.
- Support parsing DIFOP packets to get config/status data.

### Changed
- Filter MOSP/DIFOP messages with two bytes instead of one byte.
- clear current cloud point when stop the instance of rs_driver.
- Recover frame_id field for C user.

### Fixed
- Fix pcl point cloud message. Adapt it to the format of the output file of rslidar_sdk.
- Fix timestamp value of XYZIRT for RS128



## v1.5.8 2022-12-09

### Added
- Add User's guide document

### Changed
- Rename RSEOS as RSE1
- Let user's distance values cover LiDAR's

### Fixed
- Revert "Report error ERRCODE_MSOP_TIMEOUT if only DIFOP packet is received", to avoid incorrect error report.
- Fix error distance of RSM2. Change it to 250m. 



## v1.5.7 2022-10-09

### Added
- Add tool to save as PCD file
- Seperate RSBPV4 from RSBP
- Add demo app demo_online_multi_lidars 

### Changed
- Disable error report in case of wrong block id for RS128/RS80 temporarily

### Fixed
- Fix distance range of helios series. Also update distance ranges of other lidars
- Report error ERRCODE_MSOP_TIMEOUT if only DIFOP packet is received



## v1.5.6 2022-09-01

### Added
- Add option ENABLE_DOUBLE_RCVBUF to solve the packet-loss problem
- Add option ENABLE_WAIT_IF_QUEUE_EMPTY to reduce CPU usage.
- Add option ENABLE_STAMP_WITH_LOCAL to convert point cloud's timestamp to local time

### Changed 
- Make ERRCODEs different for MSOP/DIFOP Packet
- Rename error code CLOUDOVERFLOW
- For RSM2, recover its coordinate to ROS-compatible
- For RSM2, adapt to increased MSOP packet len 
- Update `demo_pcap` and `rs_driver_viewer` with cloud queue
- Accept angle.csv with vert angle only
- Update help documents

### Fixed
- For Ruby and Ruby Plus, fix the problem of parsing point cloud' timestamp.
- Fix ERRCODE_MSOPTIMEOUT problem of input_sock_epoll 

### Removed
- Remove option ENABLE_RCVMMSG



## v1.5.5 2022-08-01

### Added
- Compiled rs_driver_viewer on Windows, and add help doc
- Add option to double RECVBUF of UDP sockets

### Changed 
- Update demo_online to exit orderly.

### Fixed
- Fix runtime error of Eigen in case of ENABLE_TRANSFORM 
- Fix Compiling error on QNX
- Fix pcap_rate
- Fix the problem with repeated stop and start of driver

### Removed
- Remove option of high priority thread



## v1.5.4 2022-07-01

### Added
- Support Ruby_3.0_48
- Add option to stamp point cloud with first point

### Updated
- Distinguish 80/80v with lidar model
- Use ROS coordinate for EOS 
- Enable PCAP file parsing in default mode
- Parse DIFOP packet in case of jumbo pcap file
- Update demo_online example to use ponit cloud queue
- Update help documents

### Fixed
- Fix lidar temperature 



## v1.5.3 2022-06-01

### Added
- Add option to receive packet with epoll()
- Support Jumbo Mode

### Fixed
- Check overflow of point cloud
- Fix compiling error of multiple definition



## v1.5.2 2022-05-20

### Added
- Support RSP128/RSP80/RSP48 lidars
- Support EOS lidar
- Add option to usleep() when no packets to be handled

### Changed
- Limit error information when error happens
- Use raw buffer for packet callback
- Split frame by seq 1 (for MEMS lidars)
- Remove difop handle thread



## v1.5.1 - 2022-04-28

### Changed
- When replay MSOP/DIFOP file, use the timestamp when it is recording.
For Mechanical LiDARs,
- Split frame by block instead of by packet
- Let every point has its own timestamp, instead of using the block's one.
- 



## v1.5.0 - 2022-04-21

-- Refactory the coder part



## v1.4.6 - 2022-04-21

### Added
- Check msop timeout
- Support M2
- add cmake option ENABLE_RECVMMSG

### Changed
- Optimize point cloud transform



## v1.4.5 - 2022-03-09

### Added
- Support dense attribute
- Support to bind to a specifed ip
- Limit max size of packet queue
- Apply SO_REUSEADDR option to the receiving socket
- Support user layer and tail layer
- add macro option to disable the PCAP function.

### Changed
- Join multicast group with code instead of shell script

### Fixed
- Fix memory leaks problem
- Fix temperature calculation (for M1 only)



## v1.4.0 - 2021-11-01

### Changed
Optimazation to decrease CPU uage, includes: 
- Replace point with point cloud as template parameter
- Instead of alloc/free packet, use packet pool
- Instead of alloc/free point cloud, always keep point cloud memory
- By default, use conditional macro to disable scan_msg/camera_trigger related code



## V1.3.1

### Added
- Add vlan support
- Add somip support
- Add split frame when pkt_cnt < last_pkt_cnt in mems
- Add temperature in mems
- Add ROCK support

### Fixed
- Fix don't get time when PointType doesn't have timestamp member
- Fix ROCK light center compensation algorithm

### Removed
- Remove redundance condition code in vec.emplace_back(std::move(point)) in mech lidars



## v1.3.0 - 2020-11-10

### Added

- Add RSHELIOS support
- Add RSM1 (B3) support
- Add Windows support
- Add rs_driver_viewer, a small tool to show point cloud
- Add save_by_rows argument
- Add multi-cast support
- Add points transformation function

### Changed

- Update some decoding part for LiDARs
- Change the definition of packet message
- Update documents



## v1.2.1 - 2020-09-04

### Fixed

- Fix the timestamp calculation for RS16,RS32 & RSBP. Now the output lidar timestamp will be UTC time and will not be affected by system time zone.



## v1.2.0 - 2020-09-01

### Added

- Add interface in driver core to get lidar temperature
- Add support for point type XYZIRT (R - ring id)(T - timestamp)
- Add RS80 support
- Add interface in driver core to get camera trigger info

### Changed

- Update the decoding part for ruby in echo-dual mode
- Update the compiler version from C++11 to C++14



## v1.1.0 - 2020-07-01

### Added

- Add the limit of the length of the msop queue 
- Add the exception capture when loading .csv file

### Fixed
- Fix the bug in calculating the timestamp of 128
- Fix the bug in calculating RPM

### Changed
- Update some functions' names
- Update the program structure

### Removed
- Remove unused variables in point cloud message



## v1.0.0 - 2020-06-01

### Added 

- New program structure

- Easy to do advanced development

- Remove the redundant code in old driver.

  
