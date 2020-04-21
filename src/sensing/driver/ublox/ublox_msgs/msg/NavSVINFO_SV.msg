# see message NavSVINFO
#

uint8 chn             # Channel number, 255 for SVs not assigned to a channel
uint8 svid            # Satellite ID

uint8 flags           # Bitmask
uint8 FLAGS_SV_USED = 1                     # SV is used for navigation
uint8 FLAGS_DIFF_CORR = 2                   # Differential correction data 
                                            # is available for this SV
uint8 FLAGS_ORBIT_AVAIL = 4                 # Orbit information is available for 
                                            # this SV (Ephemeris or Almanach)
uint8 FLAGS_ORBIT_EPH = 8                   # Orbit information is Ephemeris
uint8 FLAGS_UNHEALTHY = 16                  # SV is unhealthy / shall not be 
                                            # used
uint8 FLAGS_ORBIT_ALM = 32                  # Orbit information is Almanac Plus
uint8 FLAGS_ORBIT_AOP = 64                  # Orbit information is AssistNow 
                                            # Autonomous
uint8 FLAGS_SMOOTHED = 128                  # Carrier smoothed pseudorange used

uint8 quality         # Bitfield
# qualityInd: Signal Quality indicator (range 0..7). The following list shows 
# the meaning of the different QI values:
# Note: Since IMES signals are not time synchronized, a channel tracking an IMES
# signal can never reach a quality indicator value of higher than 3.
uint8 QUALITY_IDLE = 0                      # This channel is idle
uint8 QUALITY_SEARCHING = 1                 # Channel is searching
uint8 QUALITY_ACQUIRED = 2                   # Signal acquired
uint8 QUALITY_DETECTED = 3                  # Signal detected but unusable
uint8 QUALITY_CODE_LOCK = 4                 # Code Lock on Signal
uint8 QUALITY_CODE_AND_CARRIER_LOCKED1 = 5  # Code and Carrier locked 
                                            # and time synchronized
uint8 QUALITY_CODE_AND_CARRIER_LOCKED2 = 6  # Code and Carrier locked 
                                            # and time synchronized
uint8 QUALITY_CODE_AND_CARRIER_LOCKED3 = 7  # Code and Carrier locked 
                                            # and time synchronized

uint8 cno             # Carrier to Noise Ratio (Signal Strength) [dBHz]
int8 elev             # Elevation in integer degrees [deg]
int16 azim            # Azimuth in integer degrees [deg]
int32 prRes           # Pseudo range residual in centimetres [cm]
