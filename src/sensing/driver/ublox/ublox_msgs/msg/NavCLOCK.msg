# NAV-CLOCK (0x01 0x22)
# Clock Solution
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 34

uint32 iTOW             # GPS Millisecond Time of week [ms]

int32 clkB              # Clock bias in nanoseconds [ns]
int32 clkD              # Clock drift in nanoseconds per second [ns/s]
uint32 tAcc             # Time Accuracy Estimate [ns]
uint32 fAcc             # Frequency Accuracy Estimate [ps/s]
