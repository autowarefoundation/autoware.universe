# UPD-SOS (0x09 0x14)
#
# Backup File Creation Acknowledge / System Restored from Backup
# 
# Firmware Supported on:
# u-blox 8 / u-blox M8 from protocol version 15 up to version 23.01
#

uint8 CLASS_ID = 9
uint8 MESSAGE_ID = 20

uint8 cmd                   # Command
uint8 CMD_BACKUP_CREATE_ACK = 2   # Backup File Creation Acknowledge
                                  # The message is sent from the device as 
                                  # confirmation of creation of a backup file 
                                  # in flash. The host can safely shut down the 
                                  # device after received this message.
uint8 CMD_SYSTEM_RESTORED = 3     # System Restored from Backup
                                  # The message is sent from the device to 
                                  # notify the host the BBR has been restored 
                                  # from a backup file in flash. The host 
                                  # should clear the backup file after 
                                  # receiving this message. If the UBX-UPD-SOS 
                                  # message is polled, this message will be 
                                  # present.

uint8[3] reserved0          # Reserved

uint8 response                                  # Response:
uint8 BACKUP_CREATE_NACK = 0                      # Not acknowledged
uint8 BACKUP_CREATE_ACK = 1                       # Acknowledged
uint8 SYSTEM_RESTORED_RESPONSE_UNKNOWN = 0        # Unknown
uint8 SYSTEM_RESTORED_RESPONSE_FAILED = 1         # Failed restoring from backup 
                                                  # file
uint8 SYSTEM_RESTORED_RESPONSE_RESTORED = 2       # Restored from backup file
uint8 SYSTEM_RESTORED_RESPONSE_NOT_RESTORED = 3   # Not restored (no backup)

uint8[3] reserved1          # Reserved