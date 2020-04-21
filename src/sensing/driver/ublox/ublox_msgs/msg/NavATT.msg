# NAV-ATT (0x01 0x05)
# Attitude Solution
#
# This message outputs the attitude solution as roll, pitch and heading angles.
# Supported on ADR and UDR products.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 5

uint32 iTOW               # GPS time of week of the navigation epoch [ms]
uint8 version             # Message version (0 for this version)

uint8[3] reserved0        # Reserved

int32 roll                # Vehicle roll. [deg / 1e-5]
int32 pitch               # Vehicle pitch. [deg / 1e-5]
int32 heading             # Vehicle heading. [deg / 1e-5]
uint32 accRoll            # Vehicle roll accuracy (if null, roll angle is not 
                          # available). [deg / 1e-5]
uint32 accPitch           # Vehicle pitch accuracy (if null, pitch angle is not 
                          # available). [deg / 1e-5]
uint32 accHeading         # Vehicle heading accuracy [deg / 1e-5]