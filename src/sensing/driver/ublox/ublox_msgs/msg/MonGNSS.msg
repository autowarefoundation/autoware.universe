# MON-GNSS (0x0A 0x28)
# Information message major GNSS selection
#
# This message reports major GNSS selection. Augmentation systems are not
# reported.
# 

uint8 CLASS_ID = 10
uint8 MESSAGE_ID = 40

uint8 version         # Message version (0x01 for this version)

uint8 BIT_MASK_GPS = 1
uint8 BIT_MASK_GLONASS = 2
uint8 BIT_MASK_BEIDOU = 4
uint8 BIT_MASK_GALILEO = 8

uint8 supported       # The major GNSS that can be supported by this receiver
uint8 defaultGnss     # Default major GNSS selection. If the default major GNSS 
                      # selection is currently configured in the efuse for this 
                      # receiver, it takes precedence over the default major 
                      # GNSS selection configured in the executing firmware of 
                      # this receiver.
                      # see bit mask constants
uint8 enabled         # Current major GNSS selection enabled for this receiver
                      # see bit mask constants

uint8 simultaneous    # Maximum number of concurrent major GNSS that can be 
                      # supported by this receiver

uint8[3] reserved1    # Reserved
