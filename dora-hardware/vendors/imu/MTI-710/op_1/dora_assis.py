import math


# 创建一个名为Header的类，该类具有与std_msgs/Header消息相对应的属性
class Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


# 您创建一个名为Quaternion的类，该类具有与geometry_msgs/Quaternion消息相对应的属性
class Quaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0


# 创建一个名为Vector3的类，该类具有与geometry_msgs/Vector3消息相对应的属性
class Vector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


# 创建一个名为String的类，该类具有与std_msgs/String消息相对应的属性
class String:
    def __init__(self):
        self.data = ""


class Imu:
    def __init__(self):
        self.header = Header()
        self.orientation = Quaternion()
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity = Vector3()
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration = Vector3()
        self.linear_acceleration_covariance = [0.0] * 9


# 创建一个名为TFTransformations的类，其中包含了与tf_transformations库中的函数相对应的静态方法
class TFTransformations:

    @staticmethod
    def quaternion_from_matrix(matrix):
        # 实现将旋转矩阵转换为四元数的算法
        m00 = matrix[0][0]
        m01 = matrix[0][1]
        m02 = matrix[0][2]
        m10 = matrix[1][0]
        m11 = matrix[1][1]
        m12 = matrix[1][2]
        m20 = matrix[2][0]
        m21 = matrix[2][1]
        m22 = matrix[2][2]

        qw = math.sqrt(1 + m00 + m11 + m22) / 2
        qx = (m21 - m12) / (4 * qw)
        qy = (m02 - m20) / (4 * qw)
        qz = (m10 - m01) / (4 * qw)

        return [qx, qy, qz, qw]

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        # 实现将欧拉角转换为四元数的算法
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        return [qx, qy, qz, qw]

    @staticmethod
    def identity_matrix():
        # 生成一个单位矩阵
        return [[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]]


class Parameter:
    def __init__(self, parameter_type, value):
        self.type = parameter_type
        self.value = value

    def get_fields_and_field_types(self):
        return {self.type: type(self.value).__name__}

    def get_parameter_value(self):
        return self


class My_parameter:
    def __init__(self):
        self.params = {}

    def declare_parameter(self, name, default):
        if name not in self.params:
            self.params[name] = {'type': type(default).__name__, 'value': default}

    def get_parameter(self, name):
        return self.params.get(name, None)

    def get_param(self, name, default):
        self.declare_parameter(name, default)
        parameter = self.get_parameter(name)
        parameter_type = parameter['type']
        parameter_value = parameter['value']
        return parameter_value
