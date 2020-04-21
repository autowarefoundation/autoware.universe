# CFG-ANT (0x06 0x13)
# Antenna Control Settings
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 19

uint16 flags            # Antenna Flag Mask
uint16 FLAGS_SVCS = 1         # Enable Antenna Supply Voltage Control Signal
uint16 FLAGS_SCD = 2          # Enable Short Circuit Detection
uint16 FLAGS_OCD = 4          # Enable Open Circuit Detection
uint16 FLAGS_PDWN_ON_SCD = 8  # Power Down Antenna supply if Short Circuit is 
                              # detected. (only in combination with Bit 1)
uint16 FLAGS_RECOVERY = 16    # Enable automatic recovery from short state

uint16 pins             # Antenna Pin Configuration
uint16 PIN_SWITCH_MASK = 31    # PIO-Pin used for switching antenna supply 
                               # (internal to TIM-LP/TIM-LF)
uint16 PIN_SCD_MASK = 992      # PIO-Pin used for detecting a short in the 
                               # antenna supply
uint16 PIN_OCD_MASK = 31744    # PIO-Pin used for detecting open/not connected 
                               # antenna
uint16 PIN_RECONFIG = 32678    # if set to one, and this command is sent to the 
                               # receiver, the receiver will reconfigure the 
                               # pins as specified.
