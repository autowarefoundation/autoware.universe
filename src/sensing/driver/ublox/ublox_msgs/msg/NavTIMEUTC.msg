# NAV-TIMEUTC (0x01 0x21)
# UTC Time Solution
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 33

uint32 iTOW             # GPS Millisecond time of week [ms]

uint32 tAcc             # Time Accuracy Estimate [ns]
int32 nano              # Fraction of second, range -1e9 .. 1e9 (UTC) [ns]
uint16 year             # Year, range 1999..2099 (UTC) [y]
uint8 month             # Month, range 1..12 (UTC) [month]
uint8 day               # Day of Month, range 1..31 (UTC) [d]
uint8 hour              # Hour of Day, range 0..23 (UTC) [h]
uint8 min               # Minute of Hour, range 0..59 (UTC) [min]
uint8 sec               # Seconds of Minute, range 0..60 (UTC) [s] (60 for 
                        # leap second)

uint8 valid             # Validity Flags
uint8 VALID_TOW = 1         # Valid Time of Week
uint8 VALID_WKN = 2         # Valid Week Number
uint8 VALID_UTC = 4         # Valid Leap Seconds, i.e. Leap Seconds already known
uint8 VALID_UTC_STANDARD_MASK = 240  # UTC standard Identifier Bit mask:
uint8 UTC_STANDARD_NOT_AVAILABLE = 0    # Information not available
uint8 UTC_STANDARD_CRL = 16             # Communications Research Labratory
uint8 UTC_STANDARD_NIST = 32            # National Institute of Standards and 
                                        # Technology (NIST)
uint8 UTC_STANDARD_USNO = 48            # U.S. Naval Observatory (USNO)
uint8 UTC_STANDARD_BIPM = 64            # International Bureau of Weights and 
                                        # Measures (BIPM)
uint8 UTC_STANDARD_EL = 80              # European Laboratory (tbd)
uint8 UTC_STANDARD_SU = 96              # Former Soviet Union (SU)
uint8 UTC_STANDARD_NTSC = 112           # National Time Service Center, China
uint8 UTC_STANDARD_UNKNOWN = 240