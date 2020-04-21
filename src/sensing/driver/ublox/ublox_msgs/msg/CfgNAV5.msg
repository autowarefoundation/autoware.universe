# CFG-NAV5 (0x06 0x24)
# Navigation Engine Settings

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 36

uint16 mask             # Parameters Bitmask. Only the masked
                        # parameters will be applied.
uint16 MASK_DYN = 1                 # Apply dynamic model settings
uint16 MASK_MIN_EL = 2              # Apply minimum elevation settings
uint16 MASK_FIX_MODE = 4            # Apply fix mode settings
uint16 MASK_DR_LIM = 8              # Apply DR limit settings
uint16 MASK_POS_MASK = 16           # Apply position mask settings
uint16 MASK_TIME_MASK = 32          # Apply time mask settings
uint16 MASK_STATIC_HOLD_MASK = 64   # Apply static hold settings
uint16 MASK_DGPS_MASK = 128         # Apply DGPS settings, firmware >= 7 only
uint16 MASK_CNO = 256               # Apply CNO threshold settings
uint16 MASK_UTC = 1024              # Apply UTC settings, protocol >= 16 only

uint8 dynModel          # Dynamic Platform model:
uint8 DYN_MODEL_PORTABLE = 0        # Portable
uint8 DYN_MODEL_STATIONARY = 2      # Stationary
uint8 DYN_MODEL_PEDESTRIAN = 3      # Pedestrian
uint8 DYN_MODEL_AUTOMOTIVE = 4      # Automotive
uint8 DYN_MODEL_SEA = 5             # Sea
uint8 DYN_MODEL_AIRBORNE_1G = 6     # Airborne with <1g Acceleration
uint8 DYN_MODEL_AIRBORNE_2G = 7     # Airborne with <2g Acceleration
uint8 DYN_MODEL_AIRBORNE_4G = 8     # Airborne with <4g Acceleration
uint8 DYN_MODEL_WRIST_WATCH = 9     # Wrist watch, protocol >= 18
uint8 DYN_MODEL_BIKE = 10           # Bike, protocol >= 19.2

uint8 fixMode           # Position Fixing Mode.
uint8 FIX_MODE_2D_ONLY = 1          # 2D only
uint8 FIX_MODE_3D_ONLY = 2          # 3D only
uint8 FIX_MODE_AUTO = 3             # Auto 2D/3D

int32 fixedAlt          # Fixed altitude (mean sea level) for 2D fix mode. 
                        # [m / 0.01]
uint32 fixedAltVar      # Fixed altitude variance for 2D mode. [m^2 / 0.0001]
int8 minElev            # Minimum Elevation for a GNSS satellite to be used in 
                        # NAV [deg]
uint8 drLimit           # Maximum time to perform dead reckoning [s]
                        # (linear extrapolation) in case of GPS signal loss
uint16 pDop             # Position DOP Mask to use [1 / 0.1]
uint16 tDop             # Time DOP Mask to use [1 / 0.1]
uint16 pAcc             # Position Accuracy Mask [m]
uint16 tAcc             # Time Accuracy Mask [m]
uint8 staticHoldThresh  # Static hold threshold [cm/s]
uint8 dgnssTimeOut      # DGNSS timeout, firmware 7 and newer only [s]

uint8 cnoThreshNumSvs   # Number of satellites required to have C/N0 above 
                        # cnoThresh for a fix to be attempted
uint8 cnoThresh         # C/N0 threshold for deciding whether to attempt a fix
                        # [dBHz]
uint8[2] reserved1      # Reserved

uint16 staticHoldMaxDist  # Static hold distance threshold (before quitting 
                          # static hold) [m]
uint8 utcStandard       # UTC standard to be used:
uint8 UTC_STANDARD_AUTOMATIC = 0 # receiver selects based on GNSS configuration
uint8 UTC_STANDARD_GPS = 3       # UTC as operated by the U.S. Naval Observatory 
                                 # (USNO); derived from GPS time
uint8 UTC_STANDARD_GLONASS = 6   # UTC as operated by the former Soviet Union;
                                 # derived from GLONASS time
uint8 UTC_STANDARD_BEIDOU = 7    # UTC as operated by the National Time Service
                                 # Center, China; derived from BeiDou time
uint8[5] reserved2      # Reserved