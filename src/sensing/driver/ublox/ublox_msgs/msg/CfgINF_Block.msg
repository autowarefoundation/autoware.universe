# See CfgINF message
#

uint8 protocolID          # Protocol Identifier, identifying for which
                          # protocol the configuration is set/get. The
                          # following are valid Protocol Identifiers:
                          # 0: UBX Protocol
                          # 1: NMEA Protocol
                          # 2-255: Reserved
uint8 PROTOCOL_ID_UBX = 0
uint8 PROTOCOL_ID_NMEA = 1

uint8[3] reserved1        # Reserved

uint8[6] infMsgMask       # A bit mask, saying which information messages
                          # are enabled on each I/O port
uint8 INF_MSG_ERROR = 1              # enable ERROR
uint8 INF_MSG_WARNING = 2            # enable WARNING
uint8 INF_MSG_NOTICE = 4             # enable NOTICE
uint8 INF_MSG_TEST = 8               # enable TEST
uint8 INF_MSG_DEBUG = 16             # enable DEBUG