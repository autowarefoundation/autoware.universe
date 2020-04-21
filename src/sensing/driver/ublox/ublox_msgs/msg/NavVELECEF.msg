# NAV-VELECEF (0x01 0x11)
# Velocity Solution in ECEF
#
# See important comments concerning validity of velocity given in section
# Navigation Output Filters.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 17

uint32 iTOW             # GPS Millisecond time of week [ms]

int32 ecefVX            # ECEF X velocity [cm/s]
int32 ecefVY            # ECEF Y velocity [cm/s]
int32 ecefVZ            # ECEF Z velocity [cm/s]
uint32 sAcc             # Speed Accuracy Estimate [cm/s]
