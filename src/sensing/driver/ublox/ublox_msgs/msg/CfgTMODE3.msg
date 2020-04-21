# CFG-TMODE3 (0x06, 0x71)
# Time Mode Settings 3
#
# Configures the receiver to be in Time Mode. The position referred to in this
# message is that of the Antenna Reference Point (ARP). See the Time Mode 
# Description for details.
# 
# Supported on:
#  - u-blox 8 / u-blox M8 with protocol version 20 (only with High Precision
#    GNSS products)
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 113

uint8 version                     # Message version (0x00 for this version)
uint8 reserved1                   # Reserved

uint16 flags
uint16 FLAGS_MODE_MASK = 255      # Receiver Mode:
uint16 FLAGS_MODE_DISABLED = 0      # Disabled
uint16 FLAGS_MODE_SURVEY_IN = 1     # Survey In
uint16 FLAGS_MODE_FIXED = 2         # Fixed Mode (true ARP position required)
uint16 FLAGS_LLA = 256            # Position is given in LAT/LON/ALT 
                                  # (default is ECEF)

int32 ecefXOrLat                  # WGS84 ECEF X coordinate (or latitude) of
                                  # the ARP position, depending on flags above
                                  # [cm] or [deg / 1e-7]
int32 ecefYOrLon                  # WGS84 ECEF Y coordinate (or longitude) of
                                  # the ARP position, depending on flags above
                                  # [cm] or [deg / 1e-7]
int32 ecefZOrAlt                  # WGS84 ECEF Z coordinate (or altitude) of
                                  # the ARP position, depending on flags above
                                  # [cm]
int8 ecefXOrLatHP                 # High-precision WGS84 ECEF X coordinate (or
                                  # latitude) of the ARP position, depending on
                                  # flags above. Must be in the range -99..+99.
                                  # The precise WGS84 ECEF X coordinate in units
                                  # of cm, or the precise WGS84 ECEF latitude in
                                  # units of 1e-7 degrees, is given by
                                  # ecefXOrLat + (ecefXOrLatHP * 1e-2)
                                  # [0.1 mm] or [deg * 1e-9]
int8 ecefYOrLonHP                 # High-precision WGS84 ECEF Y coordinate (or
                                  # longitude) of the ARP position, depending on
                                  # flags above. Must be in the range -99..+99.
                                  # The precise WGS84 ECEF Y coordinate in units
                                  # of cm, or the precise WGS84 ECEF longitude 
                                  # in units of 1e-7 degrees, is given by
                                  # ecefYOrLon + (ecefYOrLonHP * 1e-2)
                                  # [0.1 mm] or [deg * 1e-9]
int8 ecefZOrAltHP                 # High-precision WGS84 ECEF Z coordinate (or
                                  # altitude) of the ARP position, depending on
                                  # flags above. Must be in the range -99..+99.
                                  # The precise WGS84 ECEF Z coordinate, or
                                  # altitude coordinate, in units of cm is given
                                  # by ecefZOrAlt + (ecefZOrAltHP * 1e-2)
                                  # [0.1 mm]
uint8 reserved2                   # Reserved

uint32 fixedPosAcc                # Fixed position 3D accuracy
                                  # [0.1 mm]
uint32 svinMinDur                 # Survey-in minimum duration
                                  # [s]
uint32 svinAccLimit               # Survey-in position accuracy limit
                                  # [0.1 mm]

uint8[8] reserved3                # Reserved
