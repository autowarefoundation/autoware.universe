# NAV-SBAS (0x01 0x32)
# SBAS Status Data
#
# This message outputs the status of the SBAS sub system
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 50

uint32 iTOW             # GPS Millisecond time of week [ms]

uint8 geo               # PRN Number of the GEO where correction and integrity 
                        # data is used from

uint8 mode              # SBAS Mode
uint8 MODE_DISABLED = 0
uint8 MODE_ENABLED_INTEGRITY = 1
uint8 MODE_ENABLED_TESTMODE = 3

int8 sys                # SBAS System (WAAS/EGNOS/...)
int8 SYS_UNKNOWN = -1
int8 SYS_WAAS = 0
int8 SYS_EGNOS = 1
int8 SYS_MSAS = 2
int8 SYS_GAGAN = 3
int8 SYS_GPS = 16

uint8 service           # SBAS Services available
uint8 SERVICE_RANGING = 1
uint8 SERVICE_CORRECTIONS = 2
uint8 SERVICE_INTEGRITY = 4
uint8 SERVICE_TESTMODE = 8

uint8 cnt               # Number of SV data following
uint8[3] reserved0      # Reserved

NavSBAS_SV[] sv
