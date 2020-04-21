# CFG-MSG (0x06 0x01)
# Message Rate(s)
# 
# Set message rate for the current port

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 1

uint8 msgClass            # Message Class
uint8 msgID               # Message Identifier
uint8 rate                # Send rate on current port 
                          # [number of navigation solutions]
