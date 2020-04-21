# CFG-INF  (0x06 0x02)
# Information message configuration
#
# The value of infMsgMask[x] below are that each bit represents one of the INF 
# class messages (Bit 0 for ERROR, Bit 1 for WARNING and so on.). For a complete 
# list, see the Message Class INF. Several configurations can be concatenated to 
# one input message.
# In this case the payload length can be a multiple of the normal length. Output 
# messages from the module contain only one configuration unit. Note that I/O 
# Ports 1 and 2 correspond to serial ports 1 and 2. I/O port 0 is DDC. I/O port 
# 3 is USB. I/O port 4 is SPI. I/O port 5 is reserved for future use
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 2

# start of repeated block
CfgINF_Block[] blocks
# end of repeated block
