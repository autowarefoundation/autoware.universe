#!/usr/bin/python3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import sys
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosbag2_py import ConverterOptions
import simplekml
from simplekml import Color


class FixChecker():
    def __init__(self, bag_file):
        self.reader = SequentialReader()
        bag_storage_otions = StorageOptions(uri=bag_file, storage_id="sqlite3")
        bag_converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        self.reader.open(bag_storage_otions, bag_converter_options)

        self.type_map = {}
        for topic_type in self.reader.get_all_topics_and_types():
            self.type_map[topic_type.name] = topic_type.type

        self.kml = simplekml.Kml()
        fix_line = self.kml.newmultigeometry(name="rtk-fix(131)")
        float_line = self.kml.newmultigeometry(name="rtk-float(67)")
        fixdgps_line = self.kml.newmultigeometry(name="dgps-fix(3)")
        dgps_line = self.kml.newmultigeometry(name="dgps(2)")
        none_line = self.kml.newmultigeometry(name="none(0)")

        database = {131: fix_line, 67: float_line, 3:fixdgps_line,2:dgps_line, 0: none_line}
        database[131].style.linestyle.color = Color.blue
        database[67].style.linestyle.color = Color.green
        database[3].style.linestyle.color = Color.red
        database[2].style.linestyle.color = Color.orange
        database[0].style.linestyle.color = Color.magenta
        for d in database.values():
            d.altitudemode = simplekml.AltitudeMode.clamptoground
            d.style.linestyle.width = 5

        self.database = database

    def __call__(self, kml_file):
        last_flags = None
        coords = []
        while self.reader.has_next():
            (topic, data, _) = self.reader.read_next()
            if topic != '/sensing/gnss/ublox/navpvt':
                continue

            msg_type = get_message(self.type_map[topic])
            msg = deserialize_message(data, msg_type)
            if msg.flags != last_flags:
                if (last_flags in self.database):
                    self.database[last_flags].newlinestring(coords=coords)
                else:
                    print(last_flags)
                coords = coords[-1:]
                last_flags = msg.flags

            coords.append((msg.lon*1e-7, msg.lat*1e-7, msg.height*1e-3))

        self.kml.save(kml_file)


def main():
    if len(sys.argv) == 1:
        return
    bag_file = sys.argv[1]
    kml_file = 'fix.kml' if len(sys.argv) == 2 else sys.argv[2]

    checker = FixChecker(bag_file)
    checker(kml_file)


if __name__ == "__main__":
    main()
