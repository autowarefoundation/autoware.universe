import re
import calendar
import math
import time


# 检查NMEA句子校验和。如果通过则返回True，如果失败则返回False
def check_nmea_checksum(nmea_sentence):
    split_sentence = nmea_sentence.split("*")
    if len(split_sentence) != 2:
        # No checksum bytes were found... improperly formatted/incomplete NMEA data?
        return False
    transmitted_checksum = split_sentence[1].strip()

    # Remove the $ at the front
    data_to_checksum = split_sentence[0][1:]
    checksum = 0
    for c in data_to_checksum:
        checksum ^= ord(c)

    return ("%02X" % checksum) == transmitted_checksum.upper()


def safe_float(field):
    try:
        return float(field)
    except ValueError:
        return float("NaN")


def safe_int(field):
    try:
        return int(field)
    except ValueError:
        return 0


# 将度分格式转换为十进制格式
def convert_latitude(field):
    return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0


# 将度分格式转换为十进制格式
def convert_longitude(field):
    return safe_float(field[0:3]) + safe_float(field[3:]) / 60.0


# 读取年月日和时间转化为时间戳
def convert_time(nmea_utc):
    # Get current time in UTC for date information
    utc_struct = time.gmtime()  # immutable, so cannot modify this one
    utc_list = list(utc_struct)
    # If one of the time fields is empty, return NaN seconds
    if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
        return float("NaN")
    else:
        hours = int(nmea_utc[0:2])
        minutes = int(nmea_utc[2:4])
        seconds = int(nmea_utc[4:6])
        utc_list[3] = hours
        utc_list[4] = minutes
        utc_list[5] = seconds
        unix_time = calendar.timegm(tuple(utc_list))
        return unix_time


def convert_status_flag(status_flag):
    if status_flag == "A":
        return True
    elif status_flag == "V":
        return False
    else:
        return False


def convert_knots_to_mps(knots):
    return safe_float(knots) * 0.514444444444


# 角度转换为弧度，需要这个包装函数是因为math.radians函数不会自动转换输入。
def convert_deg_to_rads(degs):
    return math.radians(safe_float(degs))


# 解析句子所用的字典和对应的操作
"""Format for this dictionary is a sentence identifier (e.g. "GGA") as the key, with a
list of tuples where each tuple is a field name, conversion function and index
into the split sentence"""
parse_maps = {
    "CHC": [
        ("GPS_Week", int, 1),
        ("utc_time", safe_float, 2),
        ("Heading", safe_float, 3),
        ("Pitch", safe_float, 4),
        ("Roll", safe_float, 5),
        ("gyro_x", safe_float, 6),
        ("gyro_y", safe_float, 7),
        ("gyro_z", safe_float, 8),
        ("acc_x", safe_float, 9),
        ("acc_y", safe_float, 10),
        ("acc_z", safe_float, 11),
        ("Latitude", safe_float, 12),
        ("Longitude", safe_float, 13),
        ("Altitude", safe_float, 14),
        ("Ve", safe_float, 15),
        ("Vn", safe_float, 16),
        ("Vu", safe_float, 17),
        ("V", safe_float, 18),
        ("NSV1", int, 19),
        ("NSV2", int, 20),
        ("Status", int, 21),
        ("Age", int, 22),
        ("Warming", int, 23),
    ]
}


# 检查传入的 GPCHC 数据协议 是否符合特定的格式，然后解析NMEA句子的各字段
def parse_nmea_sentence(nmea_sentence):
    # 检查传入的GPCHC 数据协议 句子是否符合特定的格式
    # print("input: ",nmea_sentence)
    #parts = nmea_sentence.split('*')
    nmea_sentence= ',*'.join(nmea_sentence.split('*'))
    # print("input2: ",nmea_sentence)
    if not re.match(r"(^\$GP).*\*[0-9A-Fa-f]{2}$", nmea_sentence):
        print(
            "Regex didn't match, sentence not valid NMEA? Sentence was: %s"
            % repr(nmea_sentence)
        )
        return False
    fields = [field.strip(",") for field in nmea_sentence.split(",")]

    # Ignore the $ and talker ID portions (e.g. GP)
    sentence_type = fields[0][3:]
 
    # 解析NMEA句子的各字段
    if sentence_type not in parse_maps:
        print("Sentence type %s not in parse map, ignoring." % repr(sentence_type))
        return False

    parse_map = parse_maps[sentence_type]
    parsed_sentence = {}
    for entry in parse_map:
        parsed_sentence[entry[0]] = entry[1](fields[entry[2]])

    # 测试输出
    # for key, value in parsed_sentence.items():
    #     print(f"Key: {key}, Value: {value}")
    return {sentence_type: parsed_sentence}
