# HNR-PVT (0x28 0x00)
# High Rate Output of PVT Solution
#
# Note that during a leap second there may be more (or less) than 60 seconds in
# a minute; see the description of leap seconds for details.
#
# This message provides the position, velocity and time solution with high 
# output rate.
#
# Supported on ADR and UDR products.
#
uint8 CLASS_ID = 40
uint8 MESSAGE_ID = 0

uint32 iTOW             # GPS Millisecond time of week [ms]
uint16 year             # Year (UTC)
uint8 month             # Month, range 1..12 (UTC)
uint8 day               # Day of month, range 1..31 (UTC)
uint8 hour              # Hour of day, range 0..23 (UTC)
uint8 min               # Minute of hour, range 0..59 (UTC)
uint8 sec               # Seconds of minute, range 0..60 (UTC)

uint8 valid             # Validity flags
uint8 VALID_DATE = 1            # Valid UTC Date
uint8 VALID_TIME = 2            # Valid 
uint8 VALID_FULLY_RESOLVED = 4  # UTC time of day has been fully resolved 
                                # (no seconds uncertainty)
uint8 VALID_MAG = 8             # Valid Magnetic Declination

int32 nano              # fraction of a second [ns], range -1e9 .. 1e9 (UTC)

uint8 gpsFix            # GPS fix Type, range 0..5
uint8 FIX_TYPE_NO_FIX = 0
uint8 FIX_TYPE_DEAD_RECKONING_ONLY = 1
uint8 FIX_TYPE_2D = 2                           # Signal from only 3 SVs, 
                                                # constant altitude assumed
uint8 FIX_TYPE_3D = 3
uint8 FIX_TYPE_GPS_DEAD_RECKONING_COMBINED = 4  # GPS + Dead reckoning
uint8 FIX_TYPE_TIME_ONLY = 5                    # Time only fix 

uint8 flags             # Fix Status Flags
uint8 FLAGS_GNSS_FIX_OK = 1          # i.e. within DOP & accuracy masks
uint8 FLAGS_DIFF_SOLN = 2            # DGPS used
uint8 FLAGS_WKN_SET = 4              # Valid GPS week number
uint8 FLAGS_TOW_SET = 8              # Valid GPS time of week (iTOW & fTOW)
uint8 FLAGS_HEAD_VEH_VALID = 32      # heading of vehicle is valid

uint8[2] reserved0      # Reserved

int32 lon               # Longitude [deg / 1e-7]
int32 lat               # Latitude [deg / 1e-7]
int32 height            # Height above Ellipsoid [mm]
int32 hMSL              # Height above mean sea level [mm]

int32 gSpeed            # Ground Speed (2-D) [mm/s]
int32 speed             # Speed (3-D) [mm/s]

int32 headMot           # Heading of motion (2-D) [deg / 1e-5]
int32 headVeh           # Heading of vehicle (2-D) [deg / 1e-5]

uint32 hAcc             # Horizontal Accuracy Estimate [mm]
uint32 vAcc             # Vertical Accuracy Estimate [mm]
uint32 sAcc             # Speed Accuracy Estimate [mm/s]
uint32 headAcc          # Heading Accuracy Estimate (both motion & vehicle) 
                        # [deg / 1e-5]

uint8[4] reserved1      # Reserved