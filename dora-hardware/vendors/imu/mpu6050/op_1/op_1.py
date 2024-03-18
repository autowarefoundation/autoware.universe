import pickle
from typing import Callable
import smbus
from dora import DoraStatus


class Operator:
    """
    Template docstring
    """

    def __init__(self):
        """Called on initialisation"""
        self.GRAVITIY_MS2 = 9.80665
        self.address = 0x68
        self.bus = 1
        # self.address = address

        self.ACCEL_SCALE_MODIFIER_2G = 16384.0
        self.ACCEL_SCALE_MODIFIER_4G = 8192.0
        self.ACCEL_SCALE_MODIFIER_8G = 4096.0
        self.ACCEL_SCALE_MODIFIER_16G = 2048.0

        self.GYRO_SCALE_MODIFIER_250DEG = 131.0
        self.GYRO_SCALE_MODIFIER_500DEG = 65.5
        self.GYRO_SCALE_MODIFIER_1000DEG = 32.8
        self.GYRO_SCALE_MODIFIER_2000DEG = 16.4

        # Pre-defined ranges 加速度计量程
        self.ACCEL_RANGE_2G = 0x00
        self.ACCEL_RANGE_4G = 0x08
        self.ACCEL_RANGE_8G = 0x10
        self.ACCEL_RANGE_16G = 0x18

        # 陀螺仪量程
        self.GYRO_RANGE_250DEG = 0x00
        self.GYRO_RANGE_500DEG = 0x08
        self.GYRO_RANGE_1000DEG = 0x10
        self.GYRO_RANGE_2000DEG = 0x18

        # MPU-6050 Registers
        self.PWR_MGMT_1 = 0x6B
        self.PWR_MGMT_2 = 0x6C

        self.ACCEL_XOUT0 = 0x3B
        self.ACCEL_YOUT0 = 0x3D
        self.ACCEL_ZOUT0 = 0x3F

        self.GYRO_XOUT0 = 0x43
        self.GYRO_YOUT0 = 0x45
        self.GYRO_ZOUT0 = 0x47

        self.ACCEL_CONFIG = 0x1C
        self.GYRO_CONFIG = 0x1B
        self.MPU_CONFIG = 0x1A

        # bus被设置为1，因此IMU驱动程序将使用I2C总线1来与IMU进行通信
        self.bus = smbus.SMBus(1)
        # Wake up the MPU-6050 since it starts in sleep mode
        # self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        pass

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.

        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        # 将负数重新转换为有符号形式，以得到正确的负数值
        # 确保从I2C设备读取的有符号整数值在Python代码中得到正确的表示
        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def read_accel_range(self, raw=False):
        """Reads the range the accelerometer is set to.

        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g=False):
        """Gets and returns the X, Y and Z values from the accelerometer.

        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            # 将输入的加速度值转换为以地球表面重力加速度为单位（即m / s²）的加速度值，并返回转换后的结果
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def read_gyro_range(self, raw=False):
        """Reads the range the gyroscope is set to.

        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.

        Returns the read values in a dictionary.
        """
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier

        return {'x': x, 'y': y, 'z': z}

    def on_event(
            self,
            dora_event: dict,
            send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            accel_data = self.get_accel_data()
            gyro_data = self.get_gyro_data()
            imu_dict = {
                "accel_data": {
                    "x": accel_data['x'],
                    "y": accel_data['y'],
                    "z": accel_data['z'],
                },
                "gyro_data": {
                    "x": gyro_data['x'],
                    "y": gyro_data['y'],
                    "z": gyro_data['z'],
                }
            }
            serialized_data = pickle.dumps(imu_dict)
            send_output("Imu6050",
                        serialized_data,
                        dora_event["metadata"])
        return DoraStatus.CONTINUE

    def __del__(self):
        """Called before being deleted"""
        pass
