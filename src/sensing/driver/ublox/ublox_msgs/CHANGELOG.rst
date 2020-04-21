^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ublox_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2019-11-19)
------------------
* Add support for ZED-F9P new RELPOSNED message and provide heading output
  Fix whitespacing...
  Add RELPOSNED9 message to compile
* Add TIM product and M8U functionality as well as the TIM-TM2 message (`#27 <https://github.com/KumarRobotics/ublox/issues/27>`_)
* comment cleanup
* added MonHW message for Firmware 6.
* fixed incorrect message id
* Contributors: Evan Flynn, Ferry Schoenmakers, Veronica Lane

1.1.2 (2017-08-02)
------------------
* README and package xml updates
* added USB Cfg PRT parameters and configuration
* Added doxygen comments and made minor cleanup changes.
* Added typedefs, cleaned up formatting, and added doxygen comments. Removed try-catch statements for serializing optional blocks and used the message count to determine the size of the optional block.
* Added doxygen comments
* Changed how ACKs are handled. They are now handled through callback functions and are included in the CallbackHandlers.
* Added UPD messages.
* Added Cfg NMEA messages for firmware 6 & 7, since the messages are a different length
* Added the CfgDAT message with custom serialization for get/set (read/write) since the messages are different lengths
* corrected message comment which had incorrect units
* Added CfgNMEA message
* Added constants for NavSAT flags bitmask
* Contributors: Veronica Lane

1.1.0 (2017-07-17)
------------------
* Updated package xmls with new version number and corrected my email address. Also updated readme to include information about new version plus new parameter
* changed name of macro for clarity
* Added Cfg RST message declaration and reset function. For Firmware 8, after reconfiguring the GNSS, a cold restart is initiated.
* node now configures INF messages
* added CfgINF message
* added comment about deprecated setting
* Added NavPVT7 message since NavPVT message is a different length for firmware version 7. UbloxFirmware7Plus class now uses a template function to update diagnostics from NavPVT messages and to publish fix messages from NavPVT messages.
* Added NavATT message
* Added message declarations for newly included messages as well as serialization classes for messages with repeating blocks.
* Added all ESF messages. Added HNR cfg message and HNR PVT message. Added MGA GAL message.
* added constants for SBAS reserved and max tracking channels
* comments cleanup
* Added additional MON messages. Received INF messages are now printed to the ROS console. Also added constants and comments to serialization and a new macro so 1 message can have multiple class, message ID pairs.
* Added NavSAT message and moved subscribers for messages deprecated in version 8 to version specific subscribe methods
* Added Error message for ASIO read read errors and fixed a comment in cfg rate
* Includes BUG FIX (keep reading). Added Ublox messages (and subscribers or configuration methods + params) for High Precision GNSS devices: CfgDGNSS, NavRELPOSNED, NavSVIN. Also added subscriber & message for RxmRTCM. Changed MonVER processing, it now determines the protocol version, hardware type (e.g. SPG/HPG), and supported GNSS (e.g. Glonass, SBAS). SBAS can now be disabled on SBAS supported devices (previously SBAS settings were ignored if enable_sbas was false to prevent crashes, now it checks the MonVER message before trying to configure SBAS.
* Added CfgTMODE3 message for High Precision GNSS products
* Contributors: Veronica Lane

1.0.0 (2017-06-23)
------------------
* added myself as maintainer to package xmls and updated version numbers of modified packages.
* Baud rate and in/out protocol mask are now configurable through params and are no longer hard coded.
* Added constants for RTCM and ACK messages
* Message comment updates
* PRT message is now generic. Added constants for SPI, USB, and DDC configurations.
* Update CfgGNSS message and serialization which now publishes and receives blocks and reads and configures all GNSS settings at once. Updated MonVER message and serialization, MonVER settings are displayed during initialization, including extension chars. Changed various std::cout messages to ROS_INFO and ROS_ERROR messages.
* updated comments
* Serialization for RxmSFRBX and RxmRAWX
* Updated AID, RXM, and NAV messages to ublox 8 protocol. Added RxmSFRBX and RxmRAWX messages. Also did a 2nd pass on CFG messages for ublox 8 update. Need to serialize SFRBX.
* Fixed build errors with nav msgs and updated remaining cfg messages for firmware 8
* updated Cfg Nav 5 and X5 messages for firmware version 8
* comment cleanup for nav pvt msg
* Added Nav PVT message for protocol 8 and added publisher for ECEF messages in node.
* Fix value of GPS_TIME_ONLY_FIX constant
* Contributors: Kartik Mohta, Veronica Lane

0.0.5 (2016-08-06)
------------------

0.0.4 (2014-12-08)
------------------
* Add install targets
* Contributors: Kartik Mohta

0.0.3 (2014-10-18)
------------------
* Added MonVER, cleaned up make files a bit
* Adde c++ stuff for NAVX5 message
* Added message for NAVX5
* Added option to run in gps only mode
* Added message type for GNSS config
* Contributors: Gareth Cross

0.0.2 (2014-10-03)
------------------

0.0.1 (2014-08-15)
------------------

0.0.0 (2014-06-23)
------------------
* ublox: first commit
* Contributors: Chao Qu
