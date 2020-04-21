# MGA-GAL (0x13 0x02)
# Galileo Ephemeris Assistance
#
# This message allows the delivery of Galileo ephemeris assistance to a 
# receiver. See the description of AssistNow Online for details.
# 

uint8 CLASS_ID = 19
uint8 MESSAGE_ID = 2

uint8 type              # Message type (0x01 for this type)
uint8 version           # Message version (0x00 for this version)
uint8 svid              # Galileo Satellite identifier

uint8 reserved0         # Reserved

uint16 iodNav           # Ephemeris and clock correction issue of Data
int16 deltaN            # Mean motion difference from computed value 
                        # [semi-cir cles/s * 2^-43]
int32 m0                # Mean anomaly at reference time [semi-cir cles 2^-31]
uint32 e                # Eccentricity [2^-33]
uint32 sqrtA            # Square root of the semi-major axis [m^0.5 * 2^-19]
int32 omega0            # Longitude of ascending node of orbital plane at weekly
                        # epoch [semi-cir cles 2^-31]
int32 i0                # inclination angle at reference time 
                        # [semi-cir cles 2^-31]
int32 omega             # Argument of perigee [semi-cir cles 2^-31]
int32 omegaDot          # Rate of change of right ascension 
                        # [semi-cir cles/s 2^-43]
int16 iDot              # Rate of change of inclination angle 
                        # [semi-cir cles/s 2^-43]
int16 cuc               # Amplitude of the cosine harmonic correction term to 
                        # the argument of latitude [radians * 2^-29]
int16 cus               # Amplitude of the sine harmonic correction term to 
                        # the argument of latitude [radians * 2^-29]
int16 crc               # Amplitude of the cosine harmonic correction term 
                        # to the orbit radius [radians * 2^-5]
int16 crs               # Amplitude of the sine harmonic correction term to the 
                        # orbit radius [radians * 2^-5]
int16 cic               # Amplitude of the cosine harmonic correction term to 
                        # the angle of inclination [radians * 2^-29]
int16 cis               # Amplitude of the sine harmonic correction term to the 
                        # angle of inclination [radians * 2^-29]
uint16 toe              # Ephemeris reference time [60 * s]
int32 af0               # clock bias correction coefficient [s * 2^-34]
int32 af1               # SV clock drift correction coefficient [s/s * 2^-46]
int8 af2               # SV clock drift rate correction coefficient 
                        # [s/s^2 * 2^-59]
uint8 sisaindexE1E5b   # Signal-in-Space Accuracy index for dual frequency 
                        # E1-E5b
uint16 toc              # Clock correction data reference Time of Week [60 * s]
int16 bgdE1E5b          # E1-E5b Broadcast Group Delay

uint8[2] reserved1     # Reserved

uint8 healthE1B        # E1-B Signal Health Status
uint8 dataValidityE1B  # E1-B Data Validity Status
uint8 healthE5b        # E5b Signal Health Status
uint8 dataValidityE5b  # E5b Data Validity Status

uint8[4] reserved2     # Reserved