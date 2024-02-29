^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package teleop_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2020-10-29)
------------------

1.2.0 (2020-10-16)
------------------

1.1.0 (2020-04-21)
------------------

1.0.2 (2020-02-10)
------------------

1.0.1 (2019-09-18)
------------------

1.0.0 (2019-09-10)
------------------
* ROS2 port (`#35 <https://github.com/ros-teleop/teleop_tools/issues/35>`_)
  * ROS2 port
  * key_teleop pkg format 3
  * port teleop_tools_msgs
  * key_teleop catch KeyboardInterrupt
  * port mouse_teleop
  * add key_teleop.yaml
  * prepare tests
  * add xmllint test
  * fix xmllint tests
  * simplify joy_teleop retrieve_config
  * remove useless class KeyTeleop
  * Fixes for dynamic topic joy publishers
  - match_command() now compares button array length to the max
  deadman button index (apples to apples)
  - match_command function now checks if any of the deadman buttons
  are depressed before returning a match
  - properly handle a std_msgs/msg/Empty 'message_value' by not
  attempting to access its value
  - utilizes iter-items to correctly index into the config dict
  for 'axis_mappings''s 'axis' and 'button' values
  - set_member() now splits according to a dash (-) rather than a
  periond (.) to be consistent with ros2 param parsing & example yaml
  - adds the correct name to setup.py for test_key_teleop.py test
  * reduce copy/pasta
* Contributors: Jeremie Deray

0.3.0 (2019-01-03)
------------------

0.2.6 (2018-04-06)
------------------

0.2.5 (2017-04-21)
------------------

0.2.4 (2016-11-30)
------------------

0.2.3 (2016-07-18)
------------------

0.2.2 (2016-03-24)
------------------

0.2.1 (2016-01-29)
------------------

0.2.0 (2015-08-03)
------------------
* Update package.xmls
* Contributors: Bence Magyar

0.1.2 (2015-02-15)
------------------

0.1.1 (2014-11-17)
------------------
* Change maintainer
* Add key_teleop to metapackage
* Add teleop_tools metapackage
* Contributors: Bence Magyar, Paul Mathieu
