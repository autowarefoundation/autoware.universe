# see message RxmRAW
#

float64 cpMes             # Carrier phase measurement [L1 cycles]
float64 prMes             # Pseudorange measurement [m]
float32 doMes             # Doppler measurement [Hz]

uint8 sv                  # Space Vehicle Number
int8 mesQI                # Nav Measurements Quality Indicator
                          #  >=4 : PR+DO OK
                          #  >=5 : PR+DO+CP OK
                          #  <6 : likely loss of carrier lock in previous 
                          #       interval
int8 cno                  # Signal strength C/No. [dbHz]
uint8 lli                 # Loss of lock indicator (RINEX definition)
