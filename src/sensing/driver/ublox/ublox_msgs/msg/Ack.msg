# ACK (0x05)
# Message Acknowledged / Not-Acknowledged
#
# Output upon processing of an input message
#

uint8 CLASS_ID = 5
uint8 NACK_MESSAGE_ID = 0
uint8 ACK_MESSAGE_ID = 1

uint8 clsID   # Class ID of the (Not-)Acknowledged Message
uint8 msgID   # Message ID of the (Not-)Acknowledged Message