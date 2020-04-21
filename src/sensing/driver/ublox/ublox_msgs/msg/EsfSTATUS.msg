# ESF-STATUS (0x10 0x10)
# External Sensor Fusion (ESF) status information
#
# Supported on UDR/ADR products
#

uint8 CLASS_ID = 16
uint8 MESSAGE_ID = 16

uint32 iTOW                   # GPS time of week of the navigation epoch [ms]
uint8 version                 # Message version (2 for this version)

uint8[7] reserved1            # Reserved

uint8 fusionMode              # Fusion mode:
uint8 FUSION_MODE_INIT = 0        # receiver is initializing some unknown values
                                  # required for doing sensor fusion
uint8 FUSION_MODE_FUSION = 1      # GNSS and sensor data are 
                                  # used for navigation solution computation
uint8 FUSION_MODE_SUSPENDED = 2   # sensor fusion is temporarily disabled 
                                  # due to e.g. invalid sensor data or detected 
                                  # ferry
uint8 FUSION_MODE_DISABLED = 3    # sensor fusion is permanently disabled 
                                  # until receiver reset due e.g. to sensor 
                                  # error

uint8[2] reserved2            # Reserved

uint8 numSens                 # Number of sensors


# Start of repeated block (numSens times)
EsfSTATUS_Sens[] sens
# End of repeated block