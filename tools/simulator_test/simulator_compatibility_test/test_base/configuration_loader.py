import json
import os


class ConfigFileHandler:
    def __init__(self, file_name):
        self.current_path = os.path.dirname(os.path.realpath(__file__))
        self.file_name = self._check_file_extension(file_name)
        self.configurations = []

    def _check_file_extension(self, file_name: str):
        if file_name.endswith(".json"):
            return file_name
        else:
            raise NotImplementedError

    def set_file(self, file_name):
        if file_name.endswith(".json"):
            self.file_name = file_name
        else:
            raise NotImplementedError

    def get_file(self):
        return self.file_name

    def load(self):
        if self.file_name is None:
            self.file_name = "sensor_configurations.json"
        with open(file=self.current_path + "/" + self.file_name, mode="r") as fp:
            self.configurations = json.load(fp)

    def get_camera_list(self):
        if "Camera_list" in self.configurations.keys():
            return self.configurations["Camera_list"]
        else:
            return []

    def get_gps_list(self):
        if "GPS_list" in self.configurations.keys():
            return self.configurations["GPS_list"]
        else:
            return []

    def get_imu_list(self):
        if "IMU_list" in self.configurations.keys():
            return self.configurations["IMU_list"]
        else:
            return []

    def get_lidar_list(self):
        if "Lidar_list" in self.configurations.keys():
            return self.configurations["Lidar_list"]
        else:
            return []


if __name__ == "__main__":
    handler = ConfigFileHandler("sensor_configurations.json")
    handler.load()
    cam = handler.get_camera_list()
    gps = handler.get_gps_list()
    imu = handler.get_imu_list()
    lidar = handler.get_lidar_list()
