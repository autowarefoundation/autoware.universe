import time


class Timestamp:
    def __init__(self, sec=None, nanosec=None):
        self.sec = sec
        self.nanosec = nanosec

    @staticmethod
    def now():
        current_time = time.time()
        sec = int(current_time)
        nanosec = int((current_time - sec) * 1e9)
        return Timestamp(sec, nanosec)


class Header:
    def __init__(self):
        self.stamp = Timestamp()
        self.frame_id = ""


class DoraSentence:
    def __init__(self):
        self.header = Header()
        self.sentence = ""

    def __str__(self):
        return (
            f"DoraSentence: frame_id={self.header.frame_id}, sec={self.header.stamp.sec}, nanosec={self.header.stamp.nanosec},"
            f"sentence={self.sentence}"
        )


class NavSatStatus:
    __constants = {
        "STATUS_NO_FIX": -1,
        "STATUS_FIX": 0,
        "STATUS_SBAS_FIX": 1,
        "STATUS_GBAS_FIX": 2,
        "SERVICE_GPS": 1,
        "SERVICE_GLONASS": 2,
        "SERVICE_COMPASS": 4,
        "SERVICE_GALILEO": 8,
    }

    def STATUS_NO_FIX(self):
        """Message constant 'STATUS_NO_FIX'."""
        return self.__constants["STATUS_NO_FIX"]

    def STATUS_FIX(self):
        """Message constant 'STATUS_FIX'."""
        return self.__constants["STATUS_FIX"]

    def STATUS_SBAS_FIX(self):
        """Message constant 'STATUS_SBAS_FIX'."""
        return self.__constants["STATUS_SBAS_FIX"]

    def STATUS_GBAS_FIX(self):
        """Message constant 'STATUS_GBAS_FIX'."""
        return self.__constants["STATUS_GBAS_FIX"]

    def SERVICE_GPS(self):
        """Message constant 'SERVICE_GPS'."""
        return self.__constants["SERVICE_GPS"]

    def SERVICE_GLONASS(self):
        """Message constant 'SERVICE_GLONASS'."""
        return self.__constants["SERVICE_GLONASS"]

    def SERVICE_COMPASS(self):
        """Message constant 'SERVICE_COMPASS'."""
        return self.__constants["SERVICE_COMPASS"]

    def SERVICE_GALILEO(self):
        """Message constant 'SERVICE_GALILEO'."""
        return self.__constants["SERVICE_GALILEO"]

    def __init__(self):
        self.status = 0
        self.service = 0


class DoraNavSatFix:
    __constants = {
        "COVARIANCE_TYPE_UNKNOWN": 0,
        "COVARIANCE_TYPE_APPROXIMATED": 1,
        "COVARIANCE_TYPE_DIAGONAL_KNOWN": 2,
        "COVARIANCE_TYPE_KNOWN": 3,
    }

    def COVARIANCE_TYPE_UNKNOWN(self):
        """Message constant 'STATUS_NO_FIX'."""
        return self.__constants["COVARIANCE_TYPE_UNKNOWN"]

    def COVARIANCE_TYPE_APPROXIMATED(self):
        """Message constant 'STATUS_NO_FIX'."""
        return self.__constants["COVARIANCE_TYPE_APPROXIMATED"]

    def COVARIANCE_TYPE_DIAGONAL_KNOWN(self):
        """Message constant 'STATUS_NO_FIX'."""
        return self.__constants["COVARIANCE_TYPE_DIAGONAL_KNOWN"]

    def COVARIANCE_TYPE_KNOWN(self):
        """Message constant 'STATUS_NO_FIX'."""
        return self.__constants["COVARIANCE_TYPE_KNOWN"]

    def __init__(self):
        self.header = Header()
        self.status = NavSatStatus()
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.position_covariance = [0.0] * 9
        self.position_covariance_type = 0

    def __str__(self):
        return (
            f"DoraNavSatFix: frame_id={self.header.frame_id}, sec={self.header.stamp.sec}, nanosec={self.header.stamp.nanosec},"
            f"latitude={self.latitude},longitude={self.longitude}, altitude={self.altitude},"
            f"position_covariance={self.position_covariance}, position_covariance_type={self.position_covariance_type},"
            f"status={self.status.status},service={self.status.service}"
        )


class Quaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0


class DoraQuaternionStamped:
    def __init__(self):
        self.header = Header()
        self.quaternion = Quaternion()

    def __str__(self):
        return (
            f"DoraQuaternionStamped: frame_id={self.header.frame_id}, sec={self.header.stamp.sec}, nanosec={self.header.stamp.nanosec},"
            f"x={self.quaternion.x},y={self.quaternion.y},z={self.quaternion.z},w={self.quaternion.w}"
        )


class DoraTwistStamped:
    def __init__(self):
        self.header = Header()
        self.x_linear = 0.0
        self.y_linear = 0.0

    def __str__(self):
        return (
            f"DoraTwistStamped: frame_id={self.header.frame_id}, sec={self.header.stamp.sec}, nanosec={self.header.stamp.nanosec},"
            f"x_linear={self.x_linear},y_linear={self.y_linear}"
        )


class DoraNMEADriver:
    def __init__(self, frame_id="gps", tf_prefix="", use_RMC=False):
        self.frame_id = frame_id
        self.use_RMC = use_RMC
        self.prefix = tf_prefix
        self.current_fix = DoraNavSatFix()
        self.current_heading = DoraQuaternionStamped()
        self.current_vel = DoraTwistStamped()
        self.getParameter_NavSatStatus = NavSatStatus()
        self.getParameter_DoraNavSatFix = DoraNavSatFix()
        self.current_nmea_string = DoraSentence()

        self.valid_fix = False

        # epe = estimated position error
        self.default_epe_quality0 = 1000000
        self.default_epe_quality1 = 4.0
        self.default_epe_quality2 = 0.1
        self.default_epe_quality4 = 0.02
        self.default_epe_quality5 = 4.0
        self.default_epe_quality9 = 3.0

        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """这个字典的格式是将GGA消息中的定位质量指示（fix type）作为键，
        每个条目包含一个元组，其中包括
        默认的估计位置误差、NavSatStatus值和NavSatFix协方差值。"""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                self.getParameter_NavSatStatus.STATUS_NO_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_UNKNOWN(),
            ],
            # Invalid无效定位
            0: [
                self.default_epe_quality0,
                self.getParameter_NavSatStatus.STATUS_NO_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_UNKNOWN(),
            ],
            # SPS单点定位
            1: [
                self.default_epe_quality1,
                self.getParameter_NavSatStatus.STATUS_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED(),
            ],
            # DGPS差分定位
            2: [
                self.default_epe_quality2,
                self.getParameter_NavSatStatus.STATUS_SBAS_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED(),
            ],
            # RTK Fix实时运动静态
            4: [
                self.default_epe_quality4,
                self.getParameter_NavSatStatus.STATUS_GBAS_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED(),
            ],
            # RTK Float浮点解
            5: [
                self.default_epe_quality5,
                self.getParameter_NavSatStatus.STATUS_GBAS_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED(),
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                self.getParameter_NavSatStatus.STATUS_GBAS_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED(),
            ],
        }

    def get_frame_id(self):
        if len(self.prefix):
            return "%s/%s" % (self.prefix, self.frame_id)
        return self.frame_id

