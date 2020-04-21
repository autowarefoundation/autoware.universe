# RXM-SFRB (0x02 0x13)
# Subframe Buffer
#
# This message reports a complete subframe of broadcast navigation data decoded 
# from a single signal. The number of data words reported in each message
# depends on the nature of the signal. See the section on Broadcast Navigation
# Data for further details.
#

uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 19

uint8 gnssId            # GNSS identifier (see Cfg GNSS for constants)

uint8 svId              # Satellite identifier within corresponding GNSS system
uint8 reserved0         # Reserved
uint8 freqId            # Only used for GLONASS: This is the frequency
                        # slot + 7 (range from 0 to 13)
uint8 numWords          # The number of data words contained in this message (up
                        # to 10, for currently supported signals)
uint8 chn               # The tracking channel number the message was received
                        # on
uint8 version           # Message version, (0x02 for this version)
uint8 reserved1         # Reserved

# Start of repeated block (numWords times)
uint32[] dwrd           # The data words
# End of repeated block