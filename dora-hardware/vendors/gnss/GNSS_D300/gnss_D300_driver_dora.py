import serial
from typing import Callable
from dora import DoraStatus
from Nmea_utils import check_nmea_checksum
from Nmea_utils import parse_nmea_sentence
from DoraNmeaDriver_utils import DoraNMEADriver, Timestamp
from transforms3d._gohlketransforms import quaternion_from_euler
import math
import pickle
import time
serial_port = "/dev/ttyUSB0"
serial_baud = 115200


class Operator:
    """
    打开串口读数据，校验、解析后，send_out解析得到的消息类型
    """

    # 打开串口读取数据
    def __init__(self):
        try:
            self.GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=1)
            self.GPS.close()
            time.sleep(0.5)
            self.GPS.open()
        except serial.SerialException as ex:
            print(
                "Could not open serial port: I/O error({0}): {1}".format(
                    ex.errno, ex.strerror
                )
            )
        # self.data = "$GNHDT,268.5019,T*1A"
        self.driver = DoraNMEADriver()

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        # 读取一行数据并去除首尾空白字符
        self.data = self.GPS.readline().strip()
        # 如果数据非空
        if self.data:
            try:
                if isinstance(self.data, bytes):
                    self.data = self.data.decode("utf-8")
                nmea_sentence = self.data
                # 校验NMEA句子
                if not check_nmea_checksum(nmea_sentence):
                    print(
                        "Received a sentence with an invalid checksum. "
                        + "Sentence was: %s" % nmea_sentence
                    )
                    return DoraStatus.CONTINUE

                # 按类型解析NMEA数据:return解析的消息类型、解析得到map(对应消息类型各字段的key-value)
                parsed_sentence = parse_nmea_sentence(nmea_sentence)

                if not parsed_sentence:
                    print(
                        "Failed to parse NMEA sentence. Sentence was: %s"
                        % nmea_sentence
                    )
                    return DoraStatus.CONTINUE

                timestamp = None
                frame_id = self.driver.get_frame_id()

                if timestamp:
                    current_time = timestamp
                else:
                    current_time = Timestamp.now()

                self.driver.current_nmea_string.header.stamp = current_time
                self.driver.current_nmea_string.header.frame_id = frame_id
                self.driver.current_nmea_string.sentence = nmea_sentence
                nmea_Publish_instance = self.driver.current_nmea_string
                # 发布DoraSentence消息
                parsed_nmea_sentence = pickle.dumps(nmea_Publish_instance)
                # print(parsed_nmea_sentence)
                send_output(
                    "DoraSentence",
                    parsed_nmea_sentence,
                    dora_input["metadata"],
                )

                if not self.driver.use_RMC and "GGA" in parsed_sentence:
                    self.driver.current_fix.header.stamp = current_time
                    self.driver.current_fix.header.frame_id = frame_id
                    self.driver.current_fix.position_covariance_type = (
                        self.driver.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED()
                    )

                    data = parsed_sentence["GGA"]
                    fix_type = data["fix_type"]
                    if not (fix_type in self.driver.gps_qualities):
                        fix_type = -1
                    self.driver.gps_qual = self.driver.gps_qualities[fix_type]
                    default_epe = self.driver.gps_qual[0]
                    self.driver.current_fix.status.status = self.driver.gps_qual[1]
                    self.driver.current_fix.position_covariance_type = (
                        self.driver.gps_qual[2]
                    )
                    if self.driver.current_fix.status.status > 0:
                        self.driver.valid_fix = True
                    else:
                        self.driver.valid_fix = False

                    self.driver.current_fix.status.service = (
                        self.driver.getParameter_NavSatStatus.SERVICE_GPS()
                    )
                    latitude = data["latitude"]
                    if data["latitude_direction"] == "S":
                        latitude = -latitude
                    self.driver.current_fix.latitude = latitude

                    longitude = data["longitude"]
                    if data["longitude_direction"] == "W":
                        longitude = -longitude
                    self.driver.current_fix.longitude = longitude

                    # 高度是相对于椭球体的，因此需要进行平均海平面的调整
                    altitude = data["altitude"] + data["mean_sea_level"]
                    self.driver.current_fix.altitude = altitude

                    # 如果没收到带有 epe的 GST 消息，就使用默认的 EPE 标准差
                    if not self.driver.using_receiver_epe or math.isnan(
                        self.lon_std_dev
                    ):
                        self.driver.lon_std_dev = default_epe
                    if not self.driver.using_receiver_epe or math.isnan(
                        self.lat_std_dev
                    ):
                        self.driver.lat_std_dev = default_epe
                    if not self.driver.using_receiver_epe or math.isnan(
                        self.alt_std_dev
                    ):
                        self.driver.alt_std_dev = default_epe * 2

                    hdop = data["hdop"]
                    self.driver.current_fix.position_covariance[0] = (
                        hdop * self.driver.lon_std_dev
                    ) ** 2
                    self.driver.current_fix.position_covariance[4] = (
                        hdop * self.driver.lat_std_dev
                    ) ** 2
                    self.driver.current_fix.position_covariance[8] = (
                        2 * hdop * self.driver.alt_std_dev
                    ) ** 2  # FIXME

                    # 发布解析得到的DoraNavSatFix消息类型
                    nmea_Publish_instance = self.driver.current_fix
                    parsed_nmea_sentence = pickle.dumps(nmea_Publish_instance)
                    send_output(
                        "DoraNavSatFix",
                        parsed_nmea_sentence,
                        dora_input["metadata"],
                    )

                elif not self.driver.use_RMC and "VTG" in parsed_sentence:
                    data = parsed_sentence["VTG"]
                    # 只有在接收到有效的GGA定位修正时，才报告VTG数据
                    if self.driver.valid_fix:
                        self.driver.current_vel.header.stamp = current_time
                        self.driver.current_vel.header.frame_id = frame_id
                        self.driver.current_vel.x_linear = data["speed"] * math.sin(
                            data["true_course"]
                        )
                        self.driver.current_vel.y_linear = data["speed"] * math.cos(
                            data["true_course"]
                        )
                        #  发布解析得到的DoraTwistStamped消息类型
                        nmea_Publish_instance = self.driver.current_vel
                        parsed_nmea_sentence = pickle.dumps(nmea_Publish_instance)
                        print(parsed_nmea_sentence)
                        send_output(
                            "DoraTwistStamped",
                            parsed_nmea_sentence,
                            dora_input["metadata"],
                        )

                elif "RMC" in parsed_sentence:
                    data = parsed_sentence["RMC"]
                    # 只有在设置了use_RMC标志(参数初始化是false)时，才发布来自RMC的修复。
                    if self.driver.use_RMC:
                        if data["fix_valid"]:
                            self.driver.current_fix.status.status = (
                                self.driver.getParameter_NavSatStatus.STATUS_FIX
                            )
                        else:
                            self.driver.current_fix.status.status = (
                                self.driver.getParameter_NavSatStatus.STATUS_NO_FIX
                            )

                        self.driver.current_fix.status.service = (
                            self.driver.getParameter_NavSatStatus.SERVICE_GPS
                        )

                        self.driver.current_fix.header.stamp = current_time
                        self.driver.current_fix.header.frame_id = frame_id
                        latitude = data["latitude"]
                        if data["latitude_direction"] == "S":
                            latitude = -latitude
                        self.driver.current_fix.latitude = latitude

                        longitude = data["longitude"]
                        if data["longitude_direction"] == "W":
                            longitude = -longitude
                        self.driver.current_fix.longitude = longitude

                        self.driver.current_fix.altitude = float("NaN")
                        self.driver.current_fix.position_covariance_type = (
                            self.driver.getParameter_DoraNavSatFix.COVARIANCE_TYPE_UNKNOWN()
                        )
                        # 发布解析得到的DoraNavSatFix消息类型
                        nmea_Publish_instance = self.driver.current_fix
                        parsed_nmea_sentence = pickle.dumps(nmea_Publish_instance)
                        send_output(
                            "DoraNavSatFix",
                            parsed_nmea_sentence,
                            dora_input["metadata"],
                        )

                    if data["fix_valid"]:
                        self.driver.current_vel.header.stamp = current_time
                        self.driver.current_vel.header.frame_id = frame_id
                        self.driver.current_vel.x_linear = data["speed"] * math.sin(
                            data["true_course"]
                        )
                        self.driver.current_vel.y_linear = data["speed"] * math.cos(
                            data["true_course"]
                        )
                        # 发布解析得到的DoraTwistStamped消息类型
                        nmea_Publish_instance = self.driver.current_vel
                        parsed_nmea_sentence = pickle.dumps(nmea_Publish_instance)
                        send_output(
                            "DoraTwistStamped",
                            parsed_nmea_sentence,
                            dora_input["metadata"],
                        )

                elif "GST" in parsed_sentence:
                    data = parsed_sentence["GST"]
                    # 如果可用，使用接收器提供的误差估计。
                    self.driver.using_receiver_epe = True
                    self.driver.lon_std_dev = data["lon_std_dev"]
                    self.driver.lat_std_dev = data["lat_std_dev"]
                    self.driver.alt_std_dev = data["alt_std_dev"]

                elif "HDT" in parsed_sentence:
                    data = parsed_sentence["HDT"]
                    if data["heading"]:
                        self.driver.current_heading.header.stamp = current_time
                        self.driver.current_heading.header.frame_id = frame_id
                        q = quaternion_from_euler(0, 0, math.radians(data["heading"]))
                        self.driver.current_heading.quaternion.x = q[1]
                        self.driver.current_heading.quaternion.y = q[2]
                        self.driver.current_heading.quaternion.z = q[3]
                        self.driver.current_heading.quaternion.w = q[0]
                        # 发布解析得到的DoraQuaternionStamped消息类型
                        nmea_Publish_instance = self.driver.current_heading
                        parsed_nmea_sentence = pickle.dumps(nmea_Publish_instance)
                        # print(parsed_nmea_sentence)
                        send_output(
                            "DoraQuaternionStamped",
                            parsed_nmea_sentence,
                            dora_input["metadata"],
                        )

                return DoraStatus.CONTINUE

            except ValueError as e:
                print(
                    "Value error, likely due to missing fields in the NMEA message. Error was: %s. "
                    % e
                )
            return DoraStatus.CONTINUE

    def drop_operator(self):
        self.GPS.close()
