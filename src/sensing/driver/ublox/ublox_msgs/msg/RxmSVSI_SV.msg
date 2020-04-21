# see message RxmSVSI
#

uint8 svid            # Satellite ID

uint8 svFlag          # Information Flags
uint8 FLAG_URA_MASK = 15      # Figure of Merit (URA) range 0..15
uint8 FLAG_HEALTHY = 16       # SV healthy flag
uint8 FLAG_EPH_VAL = 32       # Ephemeris valid
uint8 FLAG_ALM_VAL = 64       # Almanac valid
uint8 FLAG_NOT_AVAIL = 128    # SV not available

int16 azim            # Azimuth
int8 elev             # Elevation

uint8 age             # Age of Almanac and Ephemeris
uint8 AGE_ALM_MASK = 15       # Age of ALM in days offset by 4
                              # i.e. the reference time may be in the future:
                              # ageOfAlm = (age & 0x0f) - 4
uint8 AGE_EPH_MASK = 240      # Age of EPH in hours offset by 4.
                              # i.e. the reference time may be in the future:
                              # ageOfEph = ((age & 0xf0) >> 4) - 4
