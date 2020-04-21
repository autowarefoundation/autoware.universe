# see message NavDGPS

uint8 svid              # Satellite ID

uint8 flags             # Bitmask / Channel Number and Usage:
uint8 FLAGS_CHANNEL_MASK = 15   # Bitmask for channel number, range 0..15
                                # Channel numbers > 15 marked as 15
uint8 FLAGS_DGPS = 16           # DGPS Used for this SV

uint16 ageC             # Age of latest correction data [ms]
float32 prc             # Pseudo Range Correction [m]
float32 prrc            # Pseudo Range Rate Correction [m/s]

