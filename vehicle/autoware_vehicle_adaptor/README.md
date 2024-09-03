# autoware_vehicle_adaptor

## Overview

Corrects the control command by considering the vehicle dynamics.

## Input topics

| Name                  | Type                                | Description         |
| --------------------- | ----------------------------------- | ------------------- |
| `~/input/control_cmd` | autoware_control_msgs::msg::Control | input control topic |

## Output topics

| Name                   | Type                                | Description                      |
| ---------------------- | ----------------------------------- | -------------------------------- |
| `~/output/control_cmd` | autoware_control_msgs::msg::Control | compensated control output topic |

## Parameters

## Limitation
