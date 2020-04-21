# CFG-NMEA (0x06 0x17)
# NMEA protocol configuration
#
# Set/Get the NMEA protocol configuration. See section NMEA Protocol 
# Configuration for a detailed description of the configuration effects on 
# NMEA output
#
# Supported on u-blox 6 from firmware version 6.00 up to version 7.03.
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 23 

uint8 filter                  # filter flags
uint8 FILTER_POS = 1          # Disable position filtering
uint8 FILTER_MSK_POS = 2      # Disable masked position filtering
uint8 FILTER_TIME = 4         # Disable time filtering
uint8 FILTER_DATE = 8         # Disable date filtering
uint8 FILTER_SBAS_FILT = 16   # Enable SBAS filtering
uint8 FILTER_TRACK = 32       # Disable track filtering

uint8 version             # NMEA version
uint8 NMEA_VERSION_2_3 = 35     # Version 2.3
uint8 NMEA_VERSION_2_1 = 33     # Version 2.1

uint8 numSV                   # Maximum Number of SVs to report in NMEA
                              # protocol.
                              # This does not affect the receiver's operation.
                              # It only limits the number of SVs reported in
                              # NMEA mode (this might be needed with older
                              # mapping applications which only support 8- or
                              # 12-channel receivers)

uint8 flags                   # flags
uint8 FLAGS_COMPAT = 1          # enable compatibility mode.
                                # This might be needed for certain applications  
                                # when customer's NMEA parser expects a fixed 
                                # number of  digits in position coordinates
uint8 FLAGS_CONSIDER = 2        # enable considering mode
