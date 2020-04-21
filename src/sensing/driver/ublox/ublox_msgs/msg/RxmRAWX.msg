# RXM-RAWX (0x02 0x15)
# Multi-GNSS Raw Measurement Data
#
# This message contains the information needed to be able to generate a RINEX 3
# multi-GNSS observation file.
# This message contains pseudorange, Doppler, carrier phase, phase lock and
# signal quality information for GNSS satellites once signals have been
# synchronized. This message supports all active GNSS.
# 

uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 21

float64 rcvTOW          # Measurement time of week in receiver local time [s]
                        # approximately aligned to the GPS time system. The
                        # receiver local time of week number and leap second
                        # information can be used to translate the time to other 
                        # time systems. More information about the difference in
                        # time systems can be found in RINEX 3 documentation. 
                        # For a receiver operating in GLONASS only mode, UTC
                        # time can be determined by subtracting the leapS field 
                        # from GPS time regardless of whether the GPS leap
                        # seconds are valid. 
uint16 week             # GPS week number in receiver local time. [weeks]
int8 leapS              # GPS leap seconds (GPS-UTC). [s]
                        # This field represents the receiver's best knowledge of
                        # the leap seconds offset. A flag is given in the
                        # recStat bitfield to indicate if the leap seconds 
                        # are known.
uint8 numMeas           # # of measurements to follow
uint8 recStat           # Receiver tracking status bitfield
uint8 REC_STAT_LEAP_SEC = 1   # Leap seconds have been determined
uint8 REC_STAT_CLK_RESET = 2  # Clock reset applied. Typically the receiver  
                              # clock is changed in increments of integer
                              # milliseconds.
uint8 version           # Message version (0x01 for this version).
uint8[2] reserved1      # Reserved

RxmRAWX_Meas[] meas
