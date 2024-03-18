"""Constant and messages definition for MT communication."""


class DeviceState:
    """State of the device"""
    # measurement state
    Measurement = 0
    # config state
    Config = 1


class MID:
    """Values for the message id (MID)"""
    # State MID
    # Wake up procedure
    WakeUp = 0x3E
    # Wake up ack to put device in config mode
    WakeUpAck = 0x3F
    # Switch to config state
    GoToConfig = 0x30
    # Switch to measurement state
    GoToMeasurement = 0x10
    # Reset device
    Reset = 0x40

    # Informational messages
    # Request device id
    ReqDID = 0x00
    # DeviceID, 4 bytes: HH HL LH LL
    DeviceID = 0x01
    # Request product code in plain text
    ReqProductCode = 0x1C
    # Product code (max 20 bytes data)
    ProductCode = 0x1D
    # Request hardware version
    ReqHardwareVersion = 0x1E
    # Hardware version, 2 bytes: major minor
    HardwareVersion = 0x1F
    # Request firmware revision
    ReqFWRev = 0x12
    # Firmware revision, 3 bytes: major minor rev
    FirmwareRev = 0x13
    # Error message, 1 data byte
    Error = 0x42

    # Device specific messages
    # Restore factory defaults
    RestoreFactoryDef = 0x0E
    # Baudrate, 1 byte
    SetBaudrate = 0x18
    # Run the built-in self test (MTi-1/10/100 series)
    RunSelftest = 0x24
    # Self test results, 2 bytes
    SelftestAck = 0x25
    # GNSS platform setting, 2 bytes (only MTi-G-700/710 with FW1.7 or higher)
    SetGnssPlatform = 0x76
    # Error mode, 2 bytes, 0000, 0001, 0002, 0003 (default 0001)
    SetErrorMode = 0xDA
    # Transmit delay (RS485), 2 bytes, number of clock ticks (1/29.4912 MHz)
    SetTransmitDelay = 0xDC
    # Set state of OptionFlags (MTi-1/2/3), 4 + 4 bytes
    SetOptionFlags = 0x48
    # Location ID, 2 bytes, arbitrary, default is 0
    SetLocationID = 0x84

    # Synchronization messages
    # Synchronization settings (MTi-1/10/100 series only), N*12 bytes
    SetSyncSettings = 0x2C

    # Configuration messages
    # Request configuration
    ReqConfiguration = 0x0C
    # Configuration, 118 bytes
    Configuration = 0x0D
    # Sampling period (MTi/MTi-G only), 2 bytes
    SetPeriod = 0x04
    # Extended output mode (MTi-10/100), 2 bytes, bit 4 for extended UART
    SetExtOutputMode = 0x86
    # Output configuration (MTi-1/10/100 series only), N*4 bytes
    SetOutputConfiguration = 0xC0
    # Configure NMEA data output (MTi-10/100), 2 bytes
    SetStringOutputType = 0x8E
    # Set sensor of local alignment quaternion
    SetAlignmentRotation = 0xEC
    # Output mode (MTi/MTi-G only), 2 bytes
    SetOutputMode = 0xD0
    # Output settings (MTi/MTi-G only), 4 bytes
    SetOutputSettings = 0xD2
    # Skip factor (MTi/MTi-G only), 2 bytes
    SetOutputSkipFactor = 0xD4

    # Data messages
    # Request MTData message (for 65535 skip factor)
    ReqData = 0x34
    # Legacy data packet
    MTData = 0x32
    # Newer data packet (MTi-10/100 series only)
    MTData2 = 0x36

    # Filter messages
    # Reset orientation, 2 bytes
    ResetOrientation = 0xA4
    # Request or set UTC time from sensor (MTI-G and MTi-10/100 series)
    SetUTCTime = 0x60
    # Set correction ticks to UTC time
    AdjustUTCTime = 0xA8
    # UTC Time (MTI-G and MTi-10/100 series), 12 bytes
    UTCTime = 0x61
    # Request the available XKF scenarios on the device
    ReqAvailableFilterProfiles = ReqAvailableScenarios = 0x62
    # Available Scenarios
    AvailableFilterProfiles = AvailableScenarios = 0x63
    # Current XKF scenario, 2 bytes
    SetFilterProfile = SetCurrentScenario = 0x64
    # Magnitude of the gravity used for the sensor fusion mechanism, 4 bytes
    SetGravityMagnitude = 0x66
    # Latitude, Longitude and Altitude for local declination and gravity
    # (MTi-10/100 series only), 24 bytes
    SetLatLonAlt = 0x6E
    # Initiate No Rotation procedure (not on MTi-G), 2 bytes
    SetNoRotation = 0x22
    # In-run Compass Calibration (ICC) command, 1 byte
    IccCommand = 0x74


class DeprecatedMID:
    """Deprecated message Ids."""
    # Informational messages
    # Compatibility for XBus Master users
    InitMT = 0x02
    InitMTResults = 0x03
    # Request data length according to current configuration
    ReqDataLength = 0x0A
    # Data Length, 2 bytes
    DataLength = 0x0B
    # Request GPS status (MTi-G only)
    ReqGPSStatus = 0xA6
    # GPS status (MTi-G only)
    GPSStatus = 0xA7

    # Synchronization messages
    # SyncIn setting (MTi only), (1+) 2 or 4 bytes depending on request
    SetSyncInSettings = 0xD6
    # SyncOut setting (MTi/MTi-G only), (1+) 2 or 4 bytes depending on request
    SetSyncOutSettings = 0xD8

    # Configuration messages
    # Skip factor (MTi/MTi-G only), 2 bytes
    SetOutputSkipFactor = 0xD4
    # Object alignment matrix, 9*4 bytes
    SetObjectAlignment = 0xE0

    # XKF Filter messages
    # Heading (MTi only), 4 bytes
    SetHeading = 0x82
    # Lever arm of the GPSin sensor coordinates (MTi-G and MTi-700 only),
    # 3*4 bytes
    SetLeverArmGPS = 0x68
    # Magnetic declination (MTi-G only), 4 bytes
    SetMagneticDeclination = 0x6A
    # Latitude, Longitude and Altitude for local declination and gravity
    # Processing flags (not on firmware 2.2 or lower for MTi/MTi-g), 1 byte
    SetProcessingFlags = 0x20


def getName(cls, value):
    '''Return the name of the first found member of class cls with given
    value.'''
    for k, v in cls.__dict__.items():
        if v == value:
            return k
    return ''


def getMIDName(mid):
    '''Return the name of a message given the message id.'''
    name = getName(MID, mid)
    if name:
        return name
    if mid & 1:
        name = getName(MID, mid-1)
        if name:
            return name+'Ack'
    return 'unknown MID'


class Baudrates(object):
    """Baudrate information and conversion."""
    # Baudrate mapping between ID and value
    Baudrates = [
        (0x0D, 4000000),
        (0x0D, 3686400),
        (0x0C, 2000000),
        (0x80,  921600),
        (0x0A,  921600),
        (0x00,  460800),
        (0x01,  230400),
        (0x02,  115200),
        (0x03,   76800),
        (0x04,   57600),
        (0x05,   38400),
        (0x06,   28800),
        (0x07,   19200),
        (0x08,   14400),
        (0x09,    9600),
        (0x0B,    4800),
        (0x80,  921600)]

    @classmethod
    def get_BRID(cls, baudrate):
        """Get baudrate id for a given baudrate."""
        for brid, br in cls.Baudrates:
            if baudrate == br:
                return brid
        raise MTException("unsupported baudrate.")

    @classmethod
    def get_BR(cls, baudrate_id):
        """Get baudrate for a given baudrate id."""
        for brid, br in cls.Baudrates:
            if baudrate_id == brid:
                return br
        raise MTException("unknown baudrate id.")


class OutputMode:
    """Values for the output mode."""
    Temp = 0x0001
    Calib = 0x0002
    Orient = 0x0004
    Auxiliary = 0x0008
    Position = 0x0010
    Velocity = 0x0020
    Status = 0x0800
    RAWGPS = 0x1000  # supposed to be incompatible with previous
    RAW = 0x4000  # incompatible with all except RAWGPS


class OutputSettings:
    """Values for the output settings."""
    Timestamp_None = 0x00000000
    Timestamp_SampleCnt = 0x00000001
    Timestamp_UTCTime = 0x00000002
    OrientMode_Quaternion = 0x00000000
    OrientMode_Euler = 0x00000004
    OrientMode_Matrix = 0x00000008
    CalibMode_AccGyrMag = 0x00000000
    CalibMode_GyrMag = 0x00000010
    CalibMode_AccMag = 0x00000020
    CalibMode_Mag = 0x00000030
    CalibMode_AccGyr = 0x00000040
    CalibMode_Gyr = 0x00000050
    CalibMode_Acc = 0x00000060
    CalibMode_Mask = 0x00000070
    DataFormat_Float = 0x00000000
    DataFormat_12_20 = 0x00000100  # not supported yet
    DataFormat_16_32 = 0x00000200  # not supported yet
    DataFormat_Double = 0x00000300  # not supported yet
    AuxiliaryMode_NoAIN1 = 0x00000400
    AuxiliaryMode_NoAIN2 = 0x00000800
    PositionMode_LLA_WGS84 = 0x00000000
    VelocityMode_MS_XYZ = 0x00000000
    Coordinates_NED = 0x80000000


class XDIGroup:
    """Values for the XDI groups."""
    Temperature = 0x0800
    Timestamp = 0x1000
    OrientationData = 0x2000
    Pressure = 0x3000
    Acceleration = 0x4000
    Position = 0x5000
    GNSS = 0x7000
    AngularVelocity = 0x8000
    GPS = 0x8800  # deprecated mk.5
    SensorComponentReadout = 0xA000
    AnalogIn = 0xB000  # deprecated mk.3
    Magnetic = 0xC000
    Velocity = 0xD000
    Status = 0xE000


class MTException(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return self.message


class MTTimeoutException(MTException):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return 'Timeout: %s' % self.message


class MTErrorMessage(MTException):
    ErrorCodes = {
        0: "Operation was performed successfully",
        1: "No bus communication possible",
        2: "InitBus and/or SetBID are not issued",
        3: "Period sent is invalid",
        4: "The message is invalid or not implemented",
        16: "A slave did not respond to WaitForSetBID",
        17: "An incorrect answer received after WaitForSetBID",
        18: "After four bus-scans still undetected Motion Trackers",
        20: "No reply to SetBID message during SetBID procedure",
        21: "Other than SetBIDAck received",
        24: "Timer overflow - period too short to collect all data from "
            "Motion Trackers",
        25: "Motion Tracker responds with other than SlaveData message",
        26: "Total bytes of data of Motion Trackers including sample counter "
            "exceeds 255 bytes",
        27: "Timer overflows during measurement",
        28: "Timer overflows during measurement",
        29: "No correct response from Motion Tracker during measurement",
        30: "Timer overflows during measurement",
        32: "Baud rate does not comply with valid range",
        33: "An invalid parameter is supplied",
        35: "TX PC Buffer is full",
        36: "TX PC Buffer overflow, cannot fit full message",
        37: "Wireless subsystem failed",
        40: "The device generated an error, try updating the firmware",
        41: "The device generates more data than the bus communication can "
            "handle (baud rate may be too low)",
        42: "The sample buffer of the device was full during a communication "
            "outage",
        43: "The external trigger is not behaving as configured",
        44: "The sample stream detected an error in the ordering of sample "
            "data",
        45: "A dip in the power supply was detected and recovered from",
        46: "A current limiter has been activated, shutting down the device",
        47: "Device temperature is not within operational limits",
        48: "Battery level reached lower limit",
        49: "Specified filter profile ID is not available on the device or "
            "the user is trying to duplicate an existing filter profile type",
        50: "The settings stored in the device's non volatile memory are "
            "invalid",
        51: "Request for control of the device was denied",
        256: "A generic error occurred",
        257: "Operation not implemented in this version (yet)",
        258: "A timeout occurred",
        259: "Operation aborted because of no data read",
        260: "Checksum fault occurred",
        261: "No internal memory available",
        262: "The requested item was not found",
        263: "Unexpected message received (e.g. no acknowledge message "
             "received)",
        264: "Invalid id supplied",
        265: "Operation is invalid at this point",
        266: "Insufficient buffer space available",
        267: "The specified i/o device can not be opened",
        268: "The specified i/o device can not be opened",
        269: "An I/O device is already opened with this object",
        270: "End of file is reached",
        271: "A required settings file could not be opened or is missing some "
             "data",
        272: "No data is available",
        273: "Tried to change a read-only value",
        274: "Tried to supply a NULL value where it is not allowed",
        275: "Insufficient data was supplied to a function",
        276: "Busy processing, try again later",
        277: "Invalid instance called",
        278: "A trusted data stream proves to contain corrupted data",
        279: "Failure during read of settings",
        280: "Could not find any MVN-compatible hardware",
        281: "Found only one responding Xbus Master",
        282: "No xsens devices found",
        283: "One or more sensors are not where they were expected",
        284: "Not enough sensors were found",
        285: "Failure during initialization of Fusion Engine",
        286: "Something else was received than was requested",
        287: "No file opened for reading/writing",
        288: "No serial port opened for reading/writing",
        289: "No file or serial port opened for reading/writing",
        290: "A required port could not be found",
        291: "The low-level port handler failed to initialize",
        292: "A calibration routine failed",
        293: "The in-config check of the device failed",
        294: "The operation is once only and has already been performed",
        295: "The single connected device is configured as a slave",
        296: "More than one master was detected",
        297: "A device was detected that was neither master nor slave",
        298: "No master detected",
        299: "A device is not sending enough data",
        300: "The version of the object is too low for the requested "
             "operation",
        301: "The object has an unrecognised version, so it's not safe to "
             "perform the operation",
        302: "The process was aborted by an external event, usually a user "
             "action or process termination",
        303: "The requested functionality is not supported by the device",
        304: "A packet counter value was missed",
        305: "An error occurred while trying to put the device in measurement "
             "mode",
        306: "A device could not start recording",
        311: "Radio channel is in use by another system",
        312: "Motion tracker disconnected unexpectedly",
        313: "Too many motion trackers connected",
        314: "A device could not be put in config mode",
        315: "Device has gone out of range",
        316: "Device is back in range, resuming normal operation",
        317: "The device was disconnected",
        400: "The device is shutting down "
    }

    def __init__(self, code):
        self.code = code
        self.message = self.ErrorCodes.get(code,
                                           'Unknown error: 0x%02X' % code)

    def __str__(self):
        return 'Error message 0x%02X: %s' % (self.code, self.message)
