# CFG-GNSS (0x06 0x3E)
# GNSS Configuration
#
# Gets or sets the GNSS system channel sharing configuration.
# If the receiver is sent a valid new configuration, it will respond with a 
# UBX-ACK-ACK message and immediately change to the new configuration. Otherwise
# the receiver will reject the request, by issuing a UBX-ACK-NAK and continuing 
# operation with the previous configuration.
# Configuration requirements:
#  - It is necessary for at least one major GNSS to be enabled, after applying 
#    the new configuration to the current one.
#  - It is also required that at least 4 tracking channels are available to each 
#    enabled major GNSS, i.e. maxTrkCh must have a minimum value of 4 for each
#    enabled major GNSS.
#  - The number of tracking channels in use must not exceed the number of 
#    tracking channels available in hardware, and the sum of all reserved 
#    tracking channels needs to be less than or equal to the number of tracking
#    channels in use.
# Notes:
#  - To avoid cross-correlation issues, it is recommended that GPS and QZSS are
#    always both enabled or both disabled.
#  - Polling this message returns the configuration of all supported GNSS, 
#    whether enabled or not; it may also include GNSS unsupported by the 
#    particular product, but in such cases the enable flag will always be unset.
#  - See section GNSS Configuration for a discussion of the use of this message 
#    and section Satellite Numbering for a description of the GNSS IDs available
#  - Configuration specific to the GNSS system can be done via other messages 
#    (e.g. UBX-CFG-SBAS).
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 62

uint8 msgVer            # Message version (= 0 for this version)
uint8 numTrkChHw        # Number of tracking channels in hardware (read only)
uint8 numTrkChUse       # (Read only in protocol versions greater than 23)
                        # Number of tracking channels to use (<= numTrkChHw)
                        # If 0xFF, then number of tracking channels to use will 
                        # be set to numTrkChHw
uint8 numConfigBlocks   # Number of configuration blocks following

# Start of repeated block (numConfigBlocks times)
CfgGNSS_Block[] blocks
# End of repeated block