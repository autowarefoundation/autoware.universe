# see NAV-SAT message
#

uint8 gnssId      # GNSS identifier
uint8 svId        # Satellite identifier

uint8 cno         # Carrier to noise ratio (signal strength) ]dBHz
int8 elev         # Elevation (range: +/-90), unknown if out of range [deg]
int16 azim        # Azimuth (range 0-360), unknown if elevation is out of range 
                  # [deg]
int16 prRes       # Pseudo range residual [0.1 m]

uint32 flags      # Bitmask
uint32 FLAGS_QUALITY_IND_MASK = 7     # Signal quality indicator:
uint8 QUALITY_IND_NO_SIGNAL = 0                     # no signal
uint8 QUALITY_IND_SEARCHING_SIGNAL = 1              # searching signal
uint8 QUALITY_IND_SIGNAL_ACQUIRED = 2               # signal acquired
uint8 QUALITY_IND_SIGNAL_DETECTED_BUT_UNUSABLE = 3  # signal detected but 
                                                    # unusable
uint8 QUALITY_IND_CODE_LOCKED_AND_TIME_SYNC = 4     # code locked and time 
                                                    # synchronized
uint8 QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC1 = 5 # code and carrier 
                                                        # locked and time 
                                                        # synchronized, 
                                                        # quality = 1
uint8 QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC2 = 6 # code and carrier 
                                                        # locked and time 
                                                        # synchronized, 
                                                        # quality = 2
uint8 QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC3 = 7 # code and carrier 
                                                        # locked and time 
                                                        # synchronized, 
                                                        # quality = 3
# Note: Since IMES signals are not time synchronized, a channel tracking an IMES 
# signal can never reach a quality indicator value of higher than 3.
uint32 FLAGS_SV_USED = 8                      # whether SV is currently being 
                                              # used for navigation
uint32 FLAGS_HEALTH_MASK = 48                 # SV health flag:
uint32 HEALTH_UNKNOWN = 0                       # unknown
uint32 HEALTH_HEALTHY = 1                       # healthy
uint32 HEALTH_UNHEALTHY = 2                     # unhealthy
uint32 FLAGS_DIFF_CORR = 64                   # whether differential correction 
                                              # data is available for this SV
uint32 FLAGS_SMOOTHED = 128                   # whether carrier smoothed 
                                              # pseudorange used
uint32 FLAGS_ORBIT_SOURCE_MASK = 1792         # Orbit source:
uint32 ORBIT_SOURCE_UNAVAILABLE = 0             # no orbit information is 
                                              # available for this SV
uint32 ORBIT_SOURCE_EPH = 256                   # ephemeris is used
uint32 ORBIT_SOURCE_ALM = 512                   # almanac is used
uint32 ORBIT_SOURCE_ASSIST_OFFLINE = 768        # AssistNow Offline orbit is 
                                                # used
uint32 ORBIT_SOURCE_ASSIST_AUTONOMOUS = 1024    # AssistNow Autonomous orbit is 
                                                # used
uint32 ORBIT_SOURCE_OTHER1 = 1280               # other orbit information is 
                                                # used
uint32 ORBIT_SOURCE_OTHER2 = 1536               # other orbit information is 
                                                # used
uint32 ORBIT_SOURCE_OTHER3 = 1792               # other orbit information is 
                                                # used
uint32 FLAGS_EPH_AVAIL = 2048                 # whether ephemeris is available 
                                              # for this SV
uint32 FLAGS_ALM_AVAIL = 4096                 # whether almanac is available for 
                                              # this SV
uint32 FLAGS_ANO_AVAIL = 8192                 # whether AssistNow Offline data 
                                              # is available for this SV
uint32 FLAGS_AOP_AVAIL = 16384                # whether AssistNow Autonomous 
                                              # data is available for this SV
uint32 FLAGS_SBAS_CORR_USED = 65536           # whether SBAS corrections have 
                                              # been used for this SV
uint32 FLAGS_RTCM_CORR_USED = 131072          # whether RTCM corrections have 
                                              # been used for this SV
uint32 FLAGS_PR_CORR_USED = 1048576           # whether Pseudorange corrections 
                                              # have been used for this SV
uint32 FLAGS_CR_CORR_USED = 2097152           # whether Carrier range 
                                              # corrections have been used for 
                                              # this SV
uint32 FLAGS_DO_CORR_USED = 4194304           # whether Range rate (Doppler) 
                                              # corrections have been used for 
                                              # this SV