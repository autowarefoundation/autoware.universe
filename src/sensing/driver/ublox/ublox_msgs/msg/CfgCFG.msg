# CFG-CFG (0x06 0x09)
# Clear, Save and Load configurations
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 9

uint32 clearMask          # Mask with configuration sub-sections to Clear
                          # (=Load Default Configurations to Permanent
                          # Configurations in non-volatile memory)
uint32 saveMask           # Mask with configuration sub-section to Save
                          # (=Save Current Configuration to Non-volatile
                          # Memory)
uint32 loadMask           # Mask with configuration sub-sections to Load
                          # (=Load Permanent Configurations from
                          # Non-volatile Memory to Current Configurations)

uint32 MASK_IO_PORT = 1       # Communications port settings. Modifying this 
                              # sub-section results in an IO system reset. 
                              # Because of this undefined data may be output 
                              # for a short period of time after receiving the
                              # message.
uint32 MASK_MSG_CONF = 2      # Message Configuration
uint32 MASK_INF_MSG = 4       # INF Message Configuration
uint32 MASK_NAV_CONF = 8      # Navigation Configuration
uint32 MASK_RXM_CONF = 16     # Receiver Manager Configuration
uint32 MASK_SEN_CONF = 256    # Sensor Interface Configuration, protocol >= 19
uint32 MASK_RINV_CONF = 512   # Remote Inventory Configuration
uint32 MASK_ANT_CONF = 1024   # Antenna Configuration
uint32 MASK_LOG_CONF = 2048   # Logging Configuration
uint32 MASK_FTS_CONF = 4096   # FTS Configuration. Only applicable to the 
                              # FTS product variant.

uint8 deviceMask          # Mask which selects the devices for this command
uint8 DEV_BBR = 1             # device battery backed RAM
uint8 DEV_FLASH = 2           # device Flash
uint8 DEV_EEPROM = 4          # device EEPROM
uint8 DEV_SPI_FLASH = 16      # device SPI Flash
