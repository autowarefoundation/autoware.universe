^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ublox_gps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2019-11-19)
------------------
* Add support for ZED-F9P new RELPOSNED message and provide heading output
  Fix whitespacing...
  Add RELPOSNED9 message to compile
* Fix for corrupted diagnostics messages
  Before the diagnostic structs were copied, but the pointers in FrequencyStatusParams still pointed to the old/freed objects.
* Show TMODE3 diagnostics OK if disabled
  Since there is no default for TMODE3 this is a deliberate choice
* added simple (remote) logger node for raw data stream logging
* updated raw data stream logging
  + moved all global/node functions to new class RawDataStreamPa
  (raw_data_pa .h/.c)
  + changed messagetype to uint8-multiarray
  (string can not handle non-characters)
* fix `#52 <https://github.com/KumarRobotics/ublox/issues/52>`_
* FIX: overflow bug when the nano field of the NavPVT message (which is signed and can be negative) is assigned to the nsec value of a ros time stamp (which is unsigned)
* deactivated config checks for base parts, if config_on_startup is false
* Added flag `config_on_startup` to deactivate configuration of ublox.
* fixes to raw data stream
  + moving write_callback\_ before the read_callback\_, to avoid buffer copying
  (write_callback\_ == publishing ros messages and writing to file)
  + publishing empty ros message during config phase to force instantiation
  of publisher
* renamed new topic and internal variables for raw data stream
  + from raw_data_xxx to raw_data_stream_xxx
  + this is to avoid confusion with the RawDataProduct
* updated debug message for measurement rate
  (added "hz" and "cycles" as units)
* TUC-ProAut: added raw data output
  (publishing ros messages and storing to file)
* boost::posix_time::seconds constructor requires integer argument
* Add TIM product and M8U functionality as well as the TIM-TM2 message (`#27 <https://github.com/KumarRobotics/ublox/issues/27>`_)
* Initialize set_usb\_ to false by default
* Set serial port to raw mode, needed for Boost versions < 1.66.0
* Minor fixes for very old devices
* Fix potential segfault when printing Inf messages
  The Inf message strings are not null terminated, so we need to construct
  the string of the correct size from the vector of bytes instead of just
  printing using %s.
* In AsyncWorker::doClose(), close the stream instead of just cancelling operations
* Cleanup + modernize to make compatible with C++11
* Fix compilation with newer GCC and Boost
  As of now, doesn't compile with C++11 or later.
* added clear params arg
* updated config files
* added save and load configuration parameters and functions. changed how GNSS is configured & reset.
* added raw data product class and structs for frequency diagnostics
* Contributors: Chao Qu, Evan Flynn, Ferry Schoenmakers, Kartik Mohta, Michael Stoll, Peter Weissig, Stewart Worrall, Tim Clephas, Veronica Lane

1.1.2 (2017-08-02)
------------------
* README and package xml updates
* Fixed bug with enabling INF messages. Changed how messages which update fix are enabled and changed name of subscribe param namespace to publish.
* added USB Cfg PRT parameters and configuration
* Changed how I/O is initialized so that u-blox node parses the device parameter, and then calls either initializeSerial or initializeTcp in the GPS class with the appropriate parameters. Also cleaned up doxygen comments
* Added doxygen comments and made minor cleanup changes.
* Added doxygen comments
* Fixed bug with ARP Position HP conversion, which was multiplied by the wrong conversion factor. The ARP Position HP parameter is now an int8 vector instead of a float vector. Also added a getRosInt method in the node to get int8 and int16 params and changed the name of getRosParam to getRosUint.
* removing unnecessary include
* Changed how ACKs are handled. They are now handled through callback functions and are included in the CallbackHandlers.
* Created a CallbackHandlers class and migrated callbacks functionality from Gps class to the CallbackHandlers class
* Node can now save flash memory on shutdown and clear flash memory based on user params
* BUG FIX: Fix status only uses NavPVT time if the time is valid, otherwise it uses ros time. This prevents invalid times.
* added respawn params to launch file
* moved getRosParam template functions into node.h and used checkRange function for the getRosParam functions
* In config files, changed reset mode since it seems to work better
* Changed how unsigned int parameters are handled.
* Added NMEA flag params for firmware version 6 and updated readme to include NMEA params.
* Renamed cfg_gnss param namespace to gnss. Fixed bug with NMEA configuration for compat variable. Added sample config file for NMEA.
* added comments
* After resetting the device when re-configuring the GNSS, the node shuts down & must be relaunched since device address may change & serial port resets.
* Made ACK atomic since it is accessed by 2 threads (the main node & the i/o callback)
* BUG FIXES: fixed bug with waiting for acknowledgements, which wasn't timing out. Fixed bug with CfgGNSS which wasn't properly verifying the current GPS config to see if it was correct. Also added NMEA configuration functions
* debug variable is no longer static so that ublox node can set it from ROS params
* Removed ublox_version param, value is now determined by parsing MonVER. Changed name of UbloxInterface to ComponentInterface for clarity.
* Additional changes to parameters
* Moved most parameters into namespaces
* Cleaned up how parameters are check and moved the parameter parsing functions from the gps namespace to the node namespace since the node handles parameter checks. Also added CfgDAT capabilities, if dat/set param is set.
* updates to sample config files
* Change rtcm_rate parameter to a vector instead of a scalar, now each RTCM id can be set to a different rate.
* BUG FIX: Fix diagnostics num sv was displaying incorrectly. For firmware versions >=7, the flags are now compared to the constants from NavPVT not NavSOL.  Also cleaned up how the diagnostics are displayed & included units. Added Carrier Phase diagnostics for HPG rovers.
* fixed bug with file path in ublox_device.launch and updated README to include information on launch files and subscribing/configuring new messages
* Contributors: Veronica Lane

1.1.0 (2017-07-17)
------------------
* Updated package xmls with new version number and corrected my email address. Also updated readme to include information about new version plus new parameter
* Updated sample config files
* Added Cfg RST message declaration and reset function. For Firmware 8, after reconfiguring the GNSS, a cold restart is initiated.
* node now configures INF messages
* Added constants for HPG Rover Diagnostic updater. Cleaned up GPS class: made method and parameter names consistent, reordered methods for clarity, and privatized some methods.
* Added NavPVT7 message since NavPVT message is a different length for firmware version 7. UbloxFirmware7Plus class now uses a template function to update diagnostics from NavPVT messages and to publish fix messages from NavPVT messages.
* Code cleanup - clarified a function name + comments
* Implemented interface for ADR/UDR messages. Added unimplemented skeleton interface for FTS messages. Added warning message if device type was not parsed correctly from MonVER.
* Cleaned up formatting + modified debug/info statements
* Changed debug statements so that they print to ROS DEBUG console. DEBUG log level is set in main node based on value of debug ros param.
* Modified Cfg GNSS for Firmware version 7, so it configures SBAS and QZSS if supported by the device
* changed receive message error print statements to only print in debug mode
* cleaned up how the tmode state was tracked for HPG reference stations. For ublox >=8, GNSS is now only configured if the current configuration is different from the desired configuration. This prevents the need for a hard-reset and prevents survey-in mode from resetting on HPG devices with the correct configuration
* I/O initialization has been entirely migrated to the GPS class, previously it was handled in both the node and GPS class. Split the HPG class into two classes, one for the REF station and one for the rover since the configuration & params did not intersect at all.
* BUG FIX: baudrate config, serial ASIO baudrate now set correctly
* Cleaned up debug print statements + code cleanup
* Added print functions for INF messages and subscribers for new MON messages
* Added NavSAT message and moved subscribers for messages deprecated in version 8 to version specific subscribe methods
* Added a UbloxInterface class. UbloxNode and Ublox firmware and hardware specific classes implement the interface. Ublox Node contains pointers to the firmware and hardware classes and calls their functions during configuration.
  Added a skeleton class for UbloxTim which subscribes to RawX and SFRBX messages, but has unimplemented configuration and getRosParams methods
* Changed UbloxNxNode class, ublox firmware version classes with version specific methods now inherit from UbloxFirmware. Hardware specific classes inherit from UbloxHardware. UbloxNode contains instances of each and calls the appropriate functions.
* Made NodeHandle a global variable in ublox_node namespace, publish is no longer a member function. Also took out additional node handles that were created to get parameters and just used the global node handle
* BUG FIX Firmware Version 6: nav status variable was never updated, using information from nav sol instead. CODE CLEANUP: added trailing underscores to a few class member variables. Removed * 3 multiplier for covariance in version 6. Added a diagnostic function for RTCM (currently not being used, will incorporate later)
* BUG FIX: For ublox 6 changed publisher of NavPOSLLH, NavVELNED, and NavSOL to call the custom method and not the template function. Also removed NavPOSLLH, NavVELNED, and NavSOL publishers from ublox 7 & 8 since NavPVT should be used. BUG FIX: Removed hardcoded value for NumTrackChs for CfgGNSS. CODE CLEANUP: added constants for hardcoded values + additional comments.
* For High Precision GNSS: Changed the way TMODE3 & RTCM messages are configured. If in survey-in mode, it first configures the device to survey-in, then when the survey is complete enables the RTCM messages.
* Fixed bug in Wait for ACK, it now checks that the ACK is for the expected class id and message id, also changed a few debug and error messages.
* Added Error message for ASIO read read errors and fixed a comment in cfg rate
* Includes BUG FIX (keep reading). Added Ublox messages (and subscribers or configuration methods + params) for High Precision GNSS devices: CfgDGNSS, NavRELPOSNED, NavSVIN. Also added subscriber & message for RxmRTCM. Changed MonVER processing, it now determines the protocol version, hardware type (e.g. SPG/HPG), and supported GNSS (e.g. Glonass, SBAS). SBAS can now be disabled on SBAS supported devices (previously SBAS settings were ignored if enable_sbas was false to prevent crashes, now it checks the MonVER message before trying to configure SBAS.
* Removed commented out lines which were unnecessary and added error message in async worker for read errors from asio
* Contributors: Veronica Lane

1.0.0 (2017-06-23)
------------------
* added myself as maintainer to package xmls and updated version numbers of modified packages.
* Modified example launch file to include params, also added example launch which loads paramaters from yaml file
* more code cleanup
* Code cleanup of node
* Made a node class structure. An abstract class represents nodes for all firmware versions. Version nodes inherit from this node and implement version specific functions.
* add ros console include so ros error message would print
* Moved callback class functions from gps files to callback.h
* Added read lock to async worker. Read + write buffers are now lockedduring operations
* Fixed Thread safety issues with async worker. Now uses MRSW lock and each function which makes changes to shared variables acquires the lock
* BUG FIX: fixed issues in gps & node that caused run time crashes. FrequencyStatusParam arguments were in the wrong order. Reverted to old initialize method which incremently set the serial baudrate.
* added constants for hard-coded values in gps class
* Baud rate and in/out protocol mask are now configurable through params and are no longer hard coded.
* Removed hardcoded configuration values and added constants and params for these values. Fixed MonVER print warning issue. Added RTCM config function. Removed FixMode & DynamicMode enums and used constants from messages. Changed setBaudrate name to configUart1 since it was configuring all params. If enable SBAS is set to false, does not call enable SBAS (need to change this so that it calls if SBAS is available) to prevent errors for devices without SBAS. Changed std::cout statements to ROS_INFO.
* Formatting of copyright so it's <80 char and changed std::cout in Async worker to ROS_INFO messages
* Update CfgGNSS message and serialization which now publishes and receives blocks and reads and configures all GNSS settings at once. Updated MonVER message and serialization, MonVER settings are displayed during initialization, including extension chars. Changed various std::cout messages to ROS_INFO and ROS_ERROR messages.
* Updated AID, RXM, and NAV messages to ublox 8 protocol. Added RxmSFRBX and RxmRAWX messages. Also did a 2nd pass on CFG messages for ublox 8 update. Need to serialize SFRBX.
* forgot to add new files in last commit
* Publishes Fix and Fix velocity from Nav PVT messages. Fix time stamps are from Nav PVT time instead of ros time now
* Publishes fix from Nav PVT info instead of Nav Pos LLH info. No longer compatible with firmware <=6. Now uses template publish function for most messages.
* Added Nav PVT message for protocol 8 and added publisher for ECEF messages in node.
* In C++11 shared_ptr has an explicit bool conversion
* Contributors: Kartik Mohta, Veronica Lane

0.0.5 (2016-08-06)
------------------
* Various small changes
  1. package.xml use format 2
  2. change some default values in launch files and node
  3. update readme
* clang format
* Contributors: Chao Qu

0.0.4 (2014-12-08)
------------------
* Update version number to reflect merge.
* Add install targets
* Reverted default in launch file
* Contributors: Gareth Cross, Kartik Mohta

0.0.3 (2014-10-18)
------------------
* Updated readme to reflect changes
* Added hacky ublox_version parameter to handle current limitations in driver structure
* Added MonVER, cleaned up make files a bit
* Added warning for ppp
* Added method to enable PPP
* Added settings for beidou and glonass
* Added option to run in gps only mode
* Changed param in roslaunch
* Contributors: Gareth Cross

0.0.2 (2014-10-03)
------------------
* Set better default for dr_limit in launch file
* Changed launch file to match readme
* Changed meas_rate to rate
* fix frame_id default
* add an option to specify node nanme
* Update ublox_gps.launch
* Update ublox_gps.launch
* Change to node
* Fixed erroneous max delay in diagnostic settings
* Removed unused option form launch file and readme
* Added diagnostic support
* Added options to ublox node, see README for details on changes
* Contributors: Chao Qu, Gareth Cross

0.0.1 (2014-08-15)
------------------
* Making fixes for second deployment
* Contributors: Gareth Cross

0.0.0 (2014-06-23)
------------------
* ublox: first commit
* Contributors: Chao Qu
