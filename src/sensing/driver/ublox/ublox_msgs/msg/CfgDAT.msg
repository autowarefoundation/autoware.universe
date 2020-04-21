# CFG-DAT (0x06 0x06)
# Set User-defined Datum
# 
# For more information see the description of Geodetic Systems and Frames.
# 

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 6

# Only for GET, these values are not used for write
uint16 datumNum       # Datum Number: 0 = WGS84, 0xFFFF = user-defined
uint16 DATUM_NUM_WGS84 = 0
uint16 DATUM_NUM_USER = 65535

uint8[6] datumName    # ASCII String: WGS84 or USER

float64 majA          # Semi-major Axis [m]
                      # accepted range = 6,300,000.0 to 6,500,000.0 meters 
float64 flat          # 1.0 / Flattening 
                      # accepted range is 0.0 to 500.0

float32 dX            # X Axis shift at the origin [m]
                      # accepted range is +/- 5000.0 meters 
float32 dY            # Y Axis shift at the origin [m]
                      # accepted range is +/- 5000.0 meters 
float32 dZ            # Z Axis shift at the origin [m]
                      # accepted range is +/- 5000.0 meters 

float32 rotX          # Rotation about the X Axis [s]
                      # accepted range is +/- 20.0 milli-arc seconds 
float32 rotY          # Rotation about the Y Axis [s]
                      # accepted range is +/- 20.0 milli-arc seconds 
float32 rotZ          # Rotation about the Z Axis [s]
                      # accepted range is +/- 20.0 milli-arc seconds

float32 scale         # Scale change [ppm]
                      # accepted range is 0.0 to 50.0 parts per million