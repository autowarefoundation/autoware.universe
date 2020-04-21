# AID-HUI (0x0B 0x02)
# GPS Health, UTC and ionosphere parameters
#
# All UBX-AID messages are deprecated; use UBX-MGA messages instead.
# This message contains a health bit mask, UTC time and Klobuchar parameters. For more
# information on these parameters, please see the ICD-GPS-200 documentation.

uint8 CLASS_ID = 11
uint8 MESSAGE_ID = 2

uint32  health          # Bitmask, every bit represents a GPS SV (1-32). 
                        # If the bit is set the SV is healthy.
float64 utcA0           # UTC - parameter A0
float64 utcA1           # UTC - parameter A1
int32   utcTOW          # UTC - reference time of week
int16   utcWNT          # UTC - reference week number
int16   utcLS           # UTC - time difference due to leap seconds before event
int16   utcWNF          # UTC - week number when next leap second event occurs
int16   utcDN           # UTC - day of week when next leap second event occurs
int16   utcLSF          # UTC - time difference due to leap seconds after event
int16   utcSpare        # UTC - Spare to ensure structure is a multiple of 4 
                        # bytes
float32 klobA0          # Klobuchar - alpha 0 [s]
float32 klobA1          # Klobuchar - alpha 1 [s/semicircle]
float32 klobA2          # Klobuchar - alpha 2 [s/semicircle^2]
float32 klobA3          # Klobuchar - alpha 3 [s/semicircle^3]
float32 klobB0          # Klobuchar - beta 0  [s]
float32 klobB1          # Klobuchar - beta 1  [s/semicircle]
float32 klobB2          # Klobuchar - beta 2  [s/semicircle^2]
float32 klobB3          # Klobuchar - beta 3  [s/semicircle^3]
uint32 flags            # flags
uint32 FLAGS_HEALTH = 1     # Healthmask field in this message is valid
uint32 FLAGS_UTC = 2        # UTC parameter fields in this message are valid
uint32 FLAGS_KLOB = 4       # Klobuchar parameter fields in this message are 
                            # valid
