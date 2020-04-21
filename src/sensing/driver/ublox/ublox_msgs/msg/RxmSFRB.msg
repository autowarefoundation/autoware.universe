# RXM-SFRB (0x02 0x11)
# Subframe Buffer
#
# The content of one single subframe buffer
# For GPS satellites, the 10 dwrd values contain the parity checked subframe 
# data for 10 Words. Each dwrd has 24 Bits with valid data (Bits 23 to 0). The 
# remaining 8 bits (31 to 24) have an undefined value. The direction within the 
# Word is that the higher order bits are received from the SV first. Example: 
# The Preamble can be found in dwrd[0], at bit position 23 down to 16. For more 
# details on the data format please refer to the ICD-GPS-200C
# Interface document.
# For SBAS satellites, the 250 Bit message block can be found in dwrd[0] to 
# dwrd[6] for the first 224 bits. The remaining 26 bits are in dwrd[7], whereas
# Bits 25 and 24 are the last two data bits, and Bits 23 down to 0 are the
# parity bits. For more information on SBAS data format, please refer to 
# RTCA/DO-229C (MOPS), Appendix A.
#

uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 17

uint8 chn               # Channel Number
uint8 svid              # ID of Satellite transmitting Subframe
uint32[10] dwrd         # Words of Data
