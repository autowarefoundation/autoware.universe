# CFG-RST (0x06 0x04)
# Reset Receiver / Clear Backup Data Structures
#
# Don't expect this message to be acknowledged by the receiver.
#  - Newer FW version won't acknowledge this message at all.
#  - Older FW version will acknowledge this message but the acknowledge may not
#    be sent completely before the receiver is reset.
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 4 

uint16 navBbrMask # BBR Sections to clear. 
# The following Special Sets apply:
uint16 NAV_BBR_HOT_START = 0       # Hot start the device
uint16 NAV_BBR_WARM_START = 1      # Warm start the device
uint16 NAV_BBR_COLD_START = 65535  # Cold start the device
uint16 NAV_BBR_EPH = 1       # Ephemeris
uint16 NAV_BBR_ALM = 2       # Almanac
uint16 NAV_BBR_HEALTH = 4    # Health
uint16 NAV_BBR_KLOB = 8      # Klobuchar parameters
uint16 NAV_BBR_POS = 16      # Position
uint16 NAV_BBR_CLKD = 32     # Clock Drift
uint16 NAV_BBR_OSC = 64      # Oscillator Parameter
uint16 NAV_BBR_UTC = 128     # UTC Correction + GPS Leap Seconds Parameters
uint16 NAV_BBR_RTC = 256     # RTC
uint16 NAV_BBR_AOP = 32768   # Autonomous Orbit Parameters

uint8 resetMode              # Reset Type:
uint8 RESET_MODE_HW_IMMEDIATE = 0       # Hardware reset (Watchdog) immediately
uint8 RESET_MODE_SW = 1                 # Controlled Software reset
uint8 RESET_MODE_GNSS = 2               # Controlled Software reset (GNSS only)
uint8 RESET_MODE_HW_AFTER_SHUTDOWN = 4  # Hardware reset (Watchdog) after 
                                        # shutdown
uint8 RESET_MODE_GNSS_STOP = 8          # Controlled GNSS stop
uint8 RESET_MODE_GNSS_START = 9         # Controlled GNSS start

uint8 reserved1     # Reserved