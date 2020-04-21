# NAV-RELPOSNED (0x01 0x3C)
# Relative Positioning Information in NED frame
# 
# The NED frame is defined as the local topological system at the reference
# station. The relative position vector components in this message, along with
# their associated accuracies, are given in that local topological system
# This message contains the relative position vector from the Reference Station
# to the Rover, including accuracy figures, in the local topological system
# defined at the reference station
#
# Supported on:
#  - u-blox 8 / u-blox M8 from protocol version 20 up to version 23.01 (only
#    with High Precision GNSS products)
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 60

uint8 version                     # Message version (0x00 for this version)
uint8 reserved0                   # Reserved
uint16 refStationId               # Reference Station ID. Must be in the range 
                                  # 0..4095
uint32 iTow                       # GPS time of week of the navigation epoch 
                                  # [ms]

int32 relPosN                     # North component of relative position vector 
                                  # [cm]
int32 relPosE                     # East component of relative position vector
                                  # [cm]
int32 relPosD                     # Down component of relative position vector
                                  # [cm]

int8 relPosHPN                    # High-precision North component of relative
                                  # position vector. [0.1 mm]
                                  # Must be in the range -99 to +99.
                                  # The full North component of the relative
                                  # position vector, in units of cm, is given by
                                  # relPosN + (relPosHPN * 1e-2)
int8 relPosHPE                    # High-precision East component of relative
                                  # position vector. [0.1 mm]
                                  # Must be in the range -99 to +99.
                                  # The full East component of the relative 
                                  # position vector, in units of cm, is given by
                                  # relPosE + (relPosHPE * 1e-2)
int8 relPosHPD                    # High-precision Down component of relative
                                  # position vector. [0.1 mm]
                                  # Must be in the range -99 to +99.
                                  # The full Down component of the relative
                                  # position vector, in units of cm, is given by
                                  # relPosD + (relPosHPD * 1e-2)

uint8 reserved1                   # Reserved

uint32 accN                       # Accuracy of relative position North 
                                  # component [0.1 mm]
uint32 accE                       # Accuracy of relative position East component
                                  # [0.1 mm]
uint32 accD                       # Accuracy of relative position Down component
                                  # [0.1 mm]

uint32 flags
uint32 FLAGS_GNSS_FIX_OK = 1      # A valid fix (i.e within DOP & accuracy 
                                  # masks)
uint32 FLAGS_DIFF_SOLN = 2        # Set if differential corrections were applied
uint32 FLAGS_REL_POS_VALID = 4    # Set if relative position components and
                                  # accuracies are valid
uint32 FLAGS_CARR_SOLN_MASK = 24  # Carrier phase range solution status:
uint32 FLAGS_CARR_SOLN_NONE = 0     # No carrier phase range solution
uint32 FLAGS_CARR_SOLN_FLOAT = 8    # Float solution. No fixed integer carrier 
                                    # phase measurements have been used to
                                    # calculate the solution
uint32 FLAGS_CARR_SOLN_FIXED = 16   # Fixed solution. One or more fixed
                                    # integer carrier phase range measurements
                                    # have been used to calculate the solution
uint32 FLAGS_IS_MOVING = 32       # if the receiver is operating in moving 
                                  # baseline mode (not supported in protocol
                                  # versions less than 20.3)
uint32 FLAGS_REF_POS_MISS = 64    # Set if extrapolated reference position was
                                  # used to compute moving baseline solution 
                                  # this epoch (not supported in protocol
                                  # versions less than 20.3)  
uint32 FLAGS_REF_OBS_MISS = 128   # Set if extrapolated reference observations
                                  # were used to compute moving baseline
                                  # solution this epoch (not supported in
                                  # protocol versions less than 20.3)