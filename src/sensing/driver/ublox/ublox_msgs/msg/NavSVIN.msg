# NAV-SVIN (0x01 0x3B)
# Survey-in data
# 
# This message contains information about survey-in parameters.
# Supported on:
#  - u-blox 8 / u-blox M8 with protocol version 20 (only with High Precision
#    GNSS products)

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 59

uint8 version           # Message version (0x00 for this version)
uint8[3] reserved0      # Reserved

uint32 iTOW             # GPS time of week of the navigation epoch [ms]

uint32 dur              # Passed survey-in observation time [s]

int32 meanX             # Current survey-in mean position ECEF X coordinate [cm]
int32 meanY             # Current survey-in mean position ECEF Y coordinate [cm]
int32 meanZ             # Current survey-in mean position ECEF Z coordinate [cm]

int8 meanXHP            # Current high-precision survey-in mean position
                        # ECEF X coordinate. 0.1_mm 
                        # Must be in the range -99..+99.
                        # The current survey-in mean position ECEF X
                        # coordinate, in units of cm, is given by
                        # meanX + (0.01 * meanXHP)
int8 meanYHP            # Current high-precision survey-in mean position
                        # ECEF Y coordinate. [0.1 mm] 
                        # Must be in the range -99..+99.
                        # The current survey-in mean position ECEF Y
                        # coordinate, in units of cm, is given by
                        # meanY + (0.01 * meanYHP)
int8 meanZHP            # Current high-precision survey-in mean position
                        # ECEF Z coordinate. [0.1 mm] 
                        # Must be in the range -99..+99.
                        # The current survey-in mean position ECEF Z
                        # coordinate, in units of cm, is given by
                        # meanZ + (0.01 * meanZHP)

uint8 reserved1         # Reserved

uint32 meanAcc          # Current survey-in mean position accuracy [0.1 mm]
uint32 obs              # Number of position observations used during survey-in
uint8 valid             # Survey-in position validity flag, 1 = valid
                        # otherwise 0
uint8 active            # Survey-in in progress flag, 1 = in-progress
                        # otherwise 0

uint8[2] reserved3      # Reserved