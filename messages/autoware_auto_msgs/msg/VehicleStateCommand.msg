# VehicleStateCommand.msg

builtin_interfaces/Time stamp
uint8 blinker 0
uint8 headlight 0
uint8 wiper 0
uint8 gear 0
uint8 mode 0
bool hand_brake false
bool horn false

### Definitions
# Blinker
uint8 BLINKER_NO_COMMAND = 0
uint8 BLINKER_OFF = 1
uint8 BLINKER_LEFT = 2
uint8 BLINKER_RIGHT = 3
uint8 BLINKER_HAZARD = 4
# Headlight
uint8 HEADLIGHT_NO_COMMAND = 0
uint8 HEADLIGHT_OFF = 1
uint8 HEADLIGHT_ON = 2
uint8 HEADLIGHT_HIGH = 3
# Wiper
uint8 WIPER_NO_COMMAND = 0
uint8 WIPER_OFF = 1
uint8 WIPER_LOW = 2
uint8 WIPER_HIGH = 3
uint8 WIPER_CLEAN = 4
# Gear
uint8 GEAR_NO_COMMAND = 0
uint8 GEAR_DRIVE = 1
uint8 GEAR_REVERSE = 2
uint8 GEAR_PARK = 3
uint8 GEAR_LOW = 4
uint8 GEAR_NEUTRAL = 5
# Mode
uint8 MODE_NO_COMMAND = 0
uint8 MODE_AUTONOMOUS = 1
uint8 MODE_MANUAL = 2
