# autoware_command_mode_decider

## Overview

This package determines the vehicle behavior as a whole system.
The determined behavior is published to the switcher node as command mode.
The command mode is the unified concept of operation mode and MRM.
Typically, the system will select a specified operation mode when normal, and if it is not available, select an appropriate MRM.
Since the command mode selection algorithm is system-dependent, developers can inherit the base class and write any processing they want.
The default implementation supports the ability to manually request any MRM.

![peripheral-ros-graph](./doc/peripheral-ros-graph.drawio.svg)
