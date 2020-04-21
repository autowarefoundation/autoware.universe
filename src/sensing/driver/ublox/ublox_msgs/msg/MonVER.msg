# MON-VER (0x0A 0x04)
#
# Receiver/Software Version
# Returned when the version is polled.

uint8 CLASS_ID = 10
uint8 MESSAGE_ID = 4

char[30] swVersion    # Zero-terminated software version string.
char[10] hwVersion    # Zero-terminated hardware version string.

# Start of repeated block (N times)
MonVER_Extension[] extension
# End of repeated block
