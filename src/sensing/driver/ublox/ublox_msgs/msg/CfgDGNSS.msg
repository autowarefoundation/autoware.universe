# CFG-DGNSS (0x06 0x70)
# DGNSS configuration
#
# This message allows the user to configure the DGNSS configuration of the 
# receiver.
# Supported on:
#  - u-blox 8 / u-blox M8 from protocol version 20.01 up to version 23.01 (only
#    with High Precision GNSS products)

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 112

uint8 dgnssMode                 # Specifies differential mode:
uint8 DGNSS_MODE_RTK_FLOAT = 2    # RTK float: No attempts are made to fix
                                  # ambiguities.
uint8 DGNSS_MODE_RTK_FIXED = 3    # RTK fixed: Ambiguities are fixed whenever
                                  # possible.
uint8[3] reserved0              # Reserved