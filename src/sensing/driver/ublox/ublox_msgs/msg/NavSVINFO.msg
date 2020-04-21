# NAV-SVINFO (0x01 0x30)
# Space Vehicle Information
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 48

uint32 iTOW             # GPS Millisecond time of week [ms]

uint8 numCh             # Number of channels

uint8 globalFlags       # Bitmask
# Chip Hardware generation flags
uint8 CHIPGEN_ANTARIS = 0   # Antaris, Antaris 4
uint8 CHIPGEN_UBLOX5 = 1    # u-blox 5
uint8 CHIPGEN_UBLOX6 = 2    # u-blox 6
uint8 CHIPGEN_UBLOX7 = 3    # u-blox 7
uint8 CHIPGEN_UBLOX8 = 4    # u-blox 8 / u-blox M8

uint16 reserved2        # Reserved

NavSVINFO_SV[] sv
