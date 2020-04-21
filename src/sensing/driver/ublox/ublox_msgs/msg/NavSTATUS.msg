# NAV-STATUS (0x01 0x03)
# Receiver Navigation Status
#
# See important comments concerning validity of position and velocity given in
# section Navigation Output Filters.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 3

uint32 iTOW             # GPS Millisecond time of week [ms]

uint8 gpsFix            # GPSfix Type, this value does not qualify a fix as
                        # valid and within the limits. See note on flag gpsFixOk
                        # below
uint8 GPS_NO_FIX = 0
uint8 GPS_DEAD_RECKONING_ONLY = 1
uint8 GPS_2D_FIX = 2
uint8 GPS_3D_FIX = 3
uint8 GPS_GPS_DEAD_RECKONING_COMBINED = 4
uint8 GPS_TIME_ONLY_FIX = 5

uint8 flags             # Navigation Status Flags
uint8 FLAGS_GPS_FIX_OK = 1      # position & velocity valid & within DOP & ACC 
                                # Masks
uint8 FLAGS_DIFF_SOLN = 2       # Differential corrections were applied
uint8 FLAGS_WKNSET = 4          # Week Number valid
uint8 FLAGS_TOWSET = 8          # Time of Week valid

uint8 fixStat           # Fix Status Information
uint8 FIX_STAT_DIFF_CORR_MASK = 1       # 1 = differential corrections available
# map matching status:
uint8 FIX_STAT_MAP_MATCHING_MASK = 192
uint8 MAP_MATCHING_NONE = 0      # none
uint8 MAP_MATCHING_VALID = 64    # valid but not used, i.e. map matching data 
                                 # was received, but was too old
uint8 MAP_MATCHING_USED = 128    # valid and used, map matching data was applied
uint8 MAP_MATCHING_DR = 192      # valid and used, map matching data was 
                                 # applied. In case of sensor unavailability map
                                 # matching data enables dead reckoning. 
                                 # This requires map matched latitude/longitude 
                                 # or heading data.

uint8 flags2            # further information about navigation output
# power safe mode state (Only for FW version >= 7.01; undefined otherwise)
uint8 FLAGS2_PSM_STATE_MASK = 3
uint8 PSM_STATE_ACQUISITION = 0                # ACQUISITION 
                                               # [or when psm disabled]
uint8 PSM_STATE_TRACKING = 1                   # TRACKING
uint8 PSM_STATE_POWER_OPTIMIZED_TRACKING = 2   # POWER OPTIMIZED TRACKING
uint8 PSM_STATE_INACTIVE = 3                   # INACTIVE
# Note that the spoofing state value only reflects the detector state for the 
# current navigation epoch. As spoofing can be detected most easily at the 
# transition from real signal to spoofing signal, this is also where the 
# detector is triggered the most. I.e. a value of 1 - No spoofing indicated does
# not mean that the receiver is not spoofed, it #simply states that the detector
# was not triggered in this epoch.
uint8 FLAGS2_SPOOF_DET_STATE_MASK = 24 
uint8 SPOOF_DET_STATE_UNKNOWN = 0    # Unknown or deactivated
uint8 SPOOF_DET_STATE_NONE = 8       # No spoofing indicated
uint8 SPOOF_DET_STATE_SPOOFING = 16  # Spoofing indicated
uint8 SPOOF_DET_STATE_MULTIPLE = 24  # Multiple spoofing indication

uint32 ttff             # Time to first fix (millisecond time tag) [ms]
uint32 msss             # Milliseconds since Startup / Reset [ms]
