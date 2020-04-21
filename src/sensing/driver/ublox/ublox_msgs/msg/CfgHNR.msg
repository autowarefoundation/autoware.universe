# CFG-HNR (0x06 0x5C)
# High Navigation Rate Settings
#
# The u-blox receivers support high rates of navigation update up to 30 Hz. 
# The navigation solution output (NAV-HNR) will not be aligned to the top of a
# second.
# The update rate has a direct influence on the power consumption. The more
# fixes that are required, the more CPU power and communication resources are 
# required.
# For most applications a 1 Hz update rate would be sufficient.
#
# (only with ADR or UDR products)
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 92

uint8 highNavRate     # Rate of navigation solution output [Hz]
uint8[3] reserved0    # Reserved