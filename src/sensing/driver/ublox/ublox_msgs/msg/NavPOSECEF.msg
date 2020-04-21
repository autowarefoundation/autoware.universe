# NAV-POSECEF (0x01 0x01)
# Position Solution in ECEF
#
# See important comments concerning validity of position given in section
# Navigation Output Filters.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 1

uint32 iTOW             # GPS Millisecond Time of Week [ms]

int32 ecefX             # ECEF X coordinate [cm]
int32 ecefY             # ECEF Y coordinate [cm]
int32 ecefZ             # ECEF Z coordinate [cm]
uint32 pAcc             # Position Accuracy Estimate [cm]
