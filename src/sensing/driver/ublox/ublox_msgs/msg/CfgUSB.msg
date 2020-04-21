# UBX-CFG-USB (0x06 0x1B)
# USB Configuration
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 27 

uint16 vendorID             # Only set to registered Vendor IDs.                     
                            # Changing this field requires special Host drivers.

uint16 productID            # Product ID. Changing this field requires special  
                            # Host drivers.

uint8[2] reserved1          # Reserved
uint8[2] reserved2          # Reserved

uint16 powerConsumption     # Power consumed by the device [mA]

uint16 flags                # various configuration flags (see graphic below)
uint16 FLAGS_RE_ENUM = 0       # force re-enumeration
uint16 FLAGS_POWER_MODE = 2    # self-powered (1), bus-powered (0)

int8[32] vendorString      # String containing the vendor name. 
                           # 32 ASCII bytes including 0-termination.
int8[32] productString     # String containing the product name. 
                           # 32 ASCII bytes including 0-termination.
int8[32] serialNumber      # String containing the serial number. 
                           # 32 ASCII bytes including 0-termination. 
                           # Changing the String fields requires special Host 
                           # drivers.