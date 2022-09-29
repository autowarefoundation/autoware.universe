# ROS topics: Voltage Monitor

"CMOS Battery Status" and "CMOS battey voltage" are exclusive.
Only one or the other is generated.
Which one is generated depends on the value of cmos_battery_label.

## <u>CMOS Battery Status</u>

/diagnostics/voltage_monitor: CMOS Battery Status

<b>[summary]</b>

| level | message     |
| ----- | ----------- |
| OK    | OK          |
| ERROR | LOW BATTERY |

<b>[values]</b>

| key (example)      | value (example)  |
| ------------------ | ---------------- |
| CMOS battey status | OK / LOW BATTERY |

\*key: thermal_zone[0-9] for ARM architecture.

## <u>CMOS Battery Voltage</u>

/diagnostics/voltage_monitor: CMOS battey voltage

<b>[summary]</b>

| level | message     |
| ----- | ----------- |
| OK    | OK          |
| WARN  | LOW BATTERY |
| ERROR | LOW BATTERY |

<b>[values]</b>

| key                 | value (example) |
| ------------------- | --------------- |
| CMOS battey voltage | 3.06            |
