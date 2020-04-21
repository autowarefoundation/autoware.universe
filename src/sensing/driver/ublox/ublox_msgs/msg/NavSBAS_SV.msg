# see message NavSBAS
#

uint8 svid              # SV Id
uint8 flags             # Flags for this SV
uint8 udre              # Monitoring status
uint8 svSys             # System (WAAS/EGNOS/...), same as SYS
uint8 svService         # Services available, same as SERVICE
uint8 reserved1         # Reserved
int16 prc               # Pseudo Range correction in [cm]
uint16 reserved2        # Reserved
int16 ic                # Ionosphere correction in [cm]
