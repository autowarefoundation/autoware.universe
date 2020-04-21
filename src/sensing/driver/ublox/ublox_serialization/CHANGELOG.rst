^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ublox_serialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2019-11-19)
------------------

1.1.2 (2017-08-02)
------------------
* README and package xml updates
* Fixed bug with enabling INF messages. Changed how messages which update fix are enabled and changed name of subscribe param namespace to publish.
* Added doxygen comments and made minor cleanup changes.
* Added doxygen comments
* Contributors: Veronica Lane

1.1.0 (2017-07-17)
------------------
* Updated package xmls with new version number and corrected my email address. Also updated readme to include information about new version plus new parameter
* changed name of macro for clarity
* changed receive message error print statements to only print in debug mode
* Added additional MON messages. Received INF messages are now printed to the ROS console. Also added constants and comments to serialization and a new macro so 1 message can have multiple class, message ID pairs.
* Contributors: Veronica Lane

1.0.0 (2017-06-23)
------------------
* added myself as maintainer to package xmls and updated version numbers of modified packages.
* formatting
* Formatting of copyright so it's <80 char and changed std::cout in Async worker to ROS_INFO messages
* Update CfgGNSS message and serialization which now publishes and receives blocks and reads and configures all GNSS settings at once. Updated MonVER message and serialization, MonVER settings are displayed during initialization, including extension chars. Changed various std::cout messages to ROS_INFO and ROS_ERROR messages.
* Contributors: Veronica Lane

0.0.5 (2016-08-06)
------------------

0.0.4 (2014-12-08)
------------------
* Add install targets
* Contributors: Kartik Mohta

0.0.3 (2014-10-18)
------------------

0.0.2 (2014-10-03)
------------------

0.0.1 (2014-08-15)
------------------

0.0.0 (2014-06-23)
------------------
* ublox: first commit
* Contributors: Chao Qu
