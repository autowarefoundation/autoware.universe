# VehicleStateReport.msg

builtin_interfaces/Time stamp
uint8 fuel # 0 to 100
uint8 blinker
uint8 headlight
uint8 wiper
uint8 gear
uint8 mode
bool hand_brake
bool horn

### Definitions
## Constants match that of VehicleStateCommand; 0 is reserved for NO_COMMAND
## (which is not relevant here)
# Blinker
uint8 BLINKER_OFF = 1
uint8 BLINKER_LEFT = 2
uint8 BLINKER_RIGHT = 3
uint8 BLINKER_HAZARD = 4
# Headlight
uint8 HEADLIGHT_OFF = 1
uint8 HEADLIGHT_ON = 2
uint8 HEADLIGHT_HIGH = 3
# Wiper
uint8 WIPER_OFF = 1
uint8 WIPER_LOW = 2
uint8 WIPER_HIGH = 3
uint8 WIPER_CLEAN = 4
# Gear
uint8 GEAR_DRIVE = 1
uint8 GEAR_REVERSE = 2
uint8 GEAR_PARK = 3
uint8 GEAR_LOW = 4
uint8 GEAR_NEUTRAL = 5
# Autonomous
uint8 MODE_AUTONOMOUS = 1
uint8 MODE_MANUAL = 2
uint8 MODE_DISENGAGED = 3
uint8 MODE_NOT_READY = 4
