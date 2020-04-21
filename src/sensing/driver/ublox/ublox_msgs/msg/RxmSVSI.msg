# RXM-SVSI (0x02 0x20)
# SV Status Info
#
# Status of the receiver manager knowledge about GPS Orbit Validity
#
# This message has only been retained for backwards compatibility; users are
# recommended to use the UBX-NAV-ORB message in preference.

uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 32

int32 iTOW              # GPS time of week of the navigation epoch [ms]
int16 week              # GPS week number of the navigation epoch [weeks]

uint8 numVis            # Number of visible satellites
uint8 numSV             # Number of per-SV data blocks following

RxmSVSI_SV[] sv
