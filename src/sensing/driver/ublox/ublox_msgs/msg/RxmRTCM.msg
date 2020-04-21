# RXM-RTCM (0x02 0x32)
# RTCM input status
#
# Output upon processing of an RTCM input message
# Supported on:
# - u-blox 8 / u-blox M8 from protocol version 20.01 up to version 23.01
# 

uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 50

uint8 version                 # Message version (0x02 for this version)
uint8 flags                   # RTCM input status flags
uint8 FLAGS_CRC_FAILED = 1    # 0 when RTCM message received and passed CRC 
                              # check, 1 when failed in which case refStation
                              # and msgType might be corrupted and misleading

uint8[2] reserved0            # Reserved

uint16 refStation             # Reference station ID
uint16 msgType                # Message type
