# ESF-INS (0x10 0x15)
# Vehicle dynamics information
# 
# This message outputs information about vehicle dynamics computed by the 
# Inertial Navigation System (INS) during ESF-based navigation.
# For ADR products, the output dynamics information (angular rates and 
# accelerations) is expressed with respect to the vehicle-frame.
# For UDR products, the output dynamics information (angular rates and 
# accelerations) is expressed with respect to the body-frame.
#

uint8 CLASS_ID = 16
uint8 MESSAGE_ID = 21

uint32 bitfield0      # Bitfield (see graphic below)
uint32 BITFIELD0_VERSION = 255            # Message version (1 for this version).
uint32 BITFIELD0_X_ANG_RATE_VALID = 256   # Compensated x-axis angular rate data 
                                          # validity flag
uint32 BITFIELD0_Y_ANG_RATE_VALID = 512   # Compensated y-axis angular rate data 
                                          # validity flag
uint32 BITFIELD0_Z_ANG_RATE_VALID = 1024  # Compensated z-axis angular rate data 
                                          # validity flag
uint32 BITFIELD0_X_ACCEL_VALID = 2048     # Compensated x-axis acceleration data 
                                          # validity flag
uint32 BITFIELD0_Y_ACCEL_VALID = 4096     # Compensated y-axis acceleration data 
                                          # validity flag
uint32 BITFIELD0_Z_ACCEL_VALID = 8192     # Compensated z-axis acceleration data 
                                          # validity flag

uint8[4] reserved1    # Reserved

uint32 iTOW           # GPS time of week of the navigation epoch [ms]
int32 xAngRate        # Compensated x-axis angular rate [deg/s / 1e-3]
int32 yAngRate        # Compensated y-axis angular rate [deg/s / 1e-3]
int32 zAngRate        # Compensated z-axis angular rate [deg/s / 1e-3]
int32 xAccel          # Compensated x-axis acceleration (gravity-free) [mg]
int32 yAccel          # Compensated y-axis acceleration (gravity-free) [mg]
int32 zAccel          # Compensated z-axis acceleration (gravity-free) [mg]