^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joy_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2020-10-29)
------------------

1.2.0 (2020-10-16)
------------------
* Change the file mode of the python files in joy_teleop.
  They don't need to be executable.
* Add in Python typing to joy_teleop.
  This also showed a few bugs.
* Add a test for service becoming ready.
* Add in tests for parameter failures.
* Add a test for debouncing.
  The test works by having a subscription to the output topic.
  Every time the subscription is received, we increment a counter.
  There is also a timer callback that executes every 0.1 seconds,
  publishing the 'joy' message.  Like other tests, this publication
  toggles the button on and off to avoid debouncing.  Unlike other
  tests, once we have seen one message come in, we stop toggling
  and just set it to one all of the time to ensure debouncing
  works properly.
* Add in a test for actions.
* Add in tests for services.
* Add a test for topic axis mappings.
* Add in a test for a simple message.
* Add in joy_teleop common testing tools.
  These will be used by the rest of the tests.
* Add in unit tests for get_interface_type.
* Rename test_pop257 -> test_pep257.
* Rewrite to use classes instead of maps.
  This has a number of benefits:
  1.  I think it is much easier to read; the classes only implement
  the pieces they are concerned with.
  2.  Actions and services now automatically reconnect as action
  servers or service servers come and go.
  3.  It is easier to write tests for individual functionality.
  The API to this node stays the same; the parameters and topics
  that were used before are still honored.
* Raise errors when parsing configuration fails.
  This lets the user know that their configuration is wrong
  much earlier.
* Get rid of self.config.
  We never use it outside of the constructor, so just make it
  a local variable.
* Rename al_clients -> action_clients.
* Contributors: Chris Lalancette

1.1.0 (2020-04-21)
------------------
* Add the ability to have deadman axes. (`#46 <https://github.com/ros-teleop/teleop_tools/issues/46>`_)
  * Add the ability to have deadman axes.
  Some controllers don't have a convenient shoulder trigger
  button, but do have shoulder "axes".  Allow the axes to
  be used for a deadman trigger, assuming they are pressed
  all the way.  Note that I used a dict for the list of
  axes, as this provides the most convenient way to deal
  with controllers that use 1.0, -1.0, or 0.0 as the "far"
  end of the axis.
  * Make sure to ignore buttons and axes that don't exist.
* Contributors: Chris Lalancette

1.0.2 (2020-02-10)
------------------
* Avoid halting on action server status checks. (`#48 <https://github.com/ros-teleop/teleop_tools/issues/48>`_)
* Depend action_tutorials_interfaces (`#44 <https://github.com/ros-teleop/teleop_tools/issues/44>`_)
* log JoyTeleopException (`#41 <https://github.com/ros-teleop/teleop_tools/issues/41>`_)
* Contributors: Michel Hidalgo, Yutaka Kondo

1.0.1 (2019-09-18)
------------------
* Fix install rules and dashing changes (`#38 <https://github.com/ros-teleop/teleop_tools/issues/38>`_)
  * fix ament indexing
  * fix package resource files
  * add tk depenndency
  * add check for param index-ability
  * data files are now package agnostic
  Signed-off-by: Ted Kern <ted.kern@canonical.com>
* Contributors: Ted Kern

1.0.0 (2019-09-10)
------------------
* ROS2 port (`#35 <https://github.com/ros-teleop/teleop_tools/issues/35>`_)
  * key_teleop pkg format 3
  * port teleop_tools_msgs
  * key_teleop catch KeyboardInterrupt
  * port mouse_teleop
  * add key_teleop.yaml
  * add xmllint test
  * fix xmllint tests
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
* Fill in the timestamp of outgoing messages, if applicable.
* add service example
* Add option for persistent service, defaulted false
* Contributors: AndyZe, Jeremie Deray, Bence Magyar

0.2.6 (2018-04-06)
------------------
* Support using buttons and axis in the same message
* Contributors: Tim Clephas

0.2.5 (2017-04-21)
------------------
* Remove duplicate examples, add list ones
* Contributors: Bence Magyar

0.2.4 (2016-11-30)
------------------
* Replace joy_teleop.fill_msg with genpy.message.fill_message_args
* Contributors: Stephen Street

0.2.3 (2016-07-18)
------------------
* Add hello publish to example
* Rename to fix example launch file
* Added example of feature to config file
* Added message_value parameter to specify message content on topics
* PEP8 style stuff
* Fixes bug when keep asking for increments
  would make the goal position grow infinitely instead of be of maximum 'current joint position' + 'increment quantity'
* Contributors: Bence Magyar, Sam Pfeiffer, SomeshDaga

0.2.2 (2016-03-24)
------------------
* Add install rules for example files
* gracefully handle missing joy axes
* Contributors: Bence Magyar, Kopias Peter

0.2.1 (2016-01-29)
------------------
* Add support for services
  it is now possible to asynchronously send service requests on button presses
* Adds queue_size keyword
* Contributors: Bence Magyar, Nils Berg, Enrique Fernandez

0.2.0 (2015-08-03)
------------------
* Add example for incrementer
* Update package.xmls
* Add incrementer_server
* Contributors: Bence Magyar

0.1.2 (2015-02-15)
------------------
* joy_teleop: fix minor typo
* Contributors: G.A. vd. Hoorn

0.1.1 (2014-11-17)
------------------
* Change maintainer
* checks for index out of bounds in buttons list
  `buttons` is a list, not a dict
  Filter out buttons not available
* Check for b in buttons
* Check for IndexError
* joy_teleop: add action server auto-refresh
* Move everything to joy_teleop subfolder
* Contributors: Bence Magyar, Enrique Fernández Perdomo, Paul Mathieu

0.1.0 (2013-11-28)
------------------
* joy_teleop: nice, generic joystick control for ROS
