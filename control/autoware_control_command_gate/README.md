# autoware_control_command_gate

## Overview

## Requirements

- Subscribe to multiple commands as command inputs.
- Generate a built-in stop command as command inputs.
- Select one of the command inputs as command output.
- Select built-in stop when selected command input is not available
- Publish command output.
- Detect timeouts for each command input.
- Filter abnormal values in command output.
- Support force stop function.

## Design

- Re-publish transient_local commands when switching.
