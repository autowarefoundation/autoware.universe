# AID-ALM (0x0B 0x30)
# GPS Aiding Almanach Input/Output Message
#
# All UBX-AID messages are deprecated; use UBX-MGA messages instead
# - If the WEEK Value is 0, DWRD0 to DWRD7 are not sent as the almanach is not 
#   available for the given SV. This may happen even if NAV-SVINFO and RXM-SVSI 
#   are indicating almanac availability as the internal data may not represent 
#   the content of an original broadcast almanac (or only parts thereof).
# - DWORD0 to DWORD7 contain the 8 words following the Hand-Over Word ( HOW )
#   from the GPS navigation message, either pages 1 to 24 of sub-frame 5 or 
#   pages 2 to 10 of subframe 4. See IS-GPS-200 for a full description of the 
#   contents of the Almanac pages.
# - In DWORD0 to DWORD7, the parity bits have been removed, and the 24 bits of 
#   data are located in Bits 0 to 23. Bits 24 to 31 shall be ignored.
# - Example: Parameter e (Eccentricity) from Almanach Subframe 4/5, Word 3, 
#   Bits 69-84 within the subframe can be found in DWRD0, Bits 15-0 whereas 
#   Bit 0 is the LSB.

uint8 CLASS_ID = 11
uint8 MESSAGE_ID = 48

uint32 svid             # SV ID for which the receiver shall return its 
                        # Almanac Data (Valid Range: 1 .. 32 or 51, 56, 63).
uint32 week             # Issue Date of Almanach (GPS week number)

# Start of optional block
uint32[] dwrd           # Almanach Words
# End of optional block
