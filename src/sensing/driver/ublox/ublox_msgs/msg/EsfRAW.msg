# ESF-RAW (0x10 0x03)
# Raw sensor measurements
#
# The message contains measurements from the active inertial sensors connected
# to the GNSS chip. Possible data types for the data field are accelerometer,
# gyroscope and temperature readings as described in the ESF Measurement Data
# section. Note that the rate selected in CFG-MSG is not respected. If a
# positive rate is selected then all raw measurements will be output. 
#
# Supported on ADR/UDR products.
#

uint8 CLASS_ID = 16
uint8 MESSAGE_ID = 3

uint8[4] reserved0 # Reserved

EsfRAW_Block[] blocks