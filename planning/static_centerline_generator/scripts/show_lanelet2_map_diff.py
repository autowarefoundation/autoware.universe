#!/bin/env python3

# Copyright 2024 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os
import subprocess
import xml.etree.ElementTree as ET


def sort_attributes(root):
    for shallow_element in root:
        attrib = shallow_element.attrib
        if len(attrib) > 1:
            # adjust attribute order, e.g. by sorting
            attribs = sorted(attrib.items())
            attrib.clear()
            attrib.update(attribs)
        if shallow_element.tag == "relation":
            pass

        # try:
        list_shallow_element = sorted(shallow_element, key=lambda x: x.items())
        for idx, val_shallow_element in enumerate(list_shallow_element):
            shallow_element[idx] = val_shallow_element
        # except:
        #     pass
        for deep_element in shallow_element:
            attrib = deep_element.attrib
            if len(attrib) > 1:
                # adjust attribute order, e.g. by sorting
                attribs = sorted(attrib.items())
                attrib.clear()
                attrib.update(attribs)


def remove_diff_to_ignore(osm_root):
    decimal_precision = 10  # for lat/lon values

    for osm_child in osm_root:
        if "visible" in osm_child.attrib and osm_child.attrib["visible"]:
            osm_child.attrib.pop("visible")
        if "version" in osm_child.attrib and osm_child.attrib["version"]:
            osm_child.attrib.pop("version")
        if "action" in osm_child.attrib and osm_child.attrib["action"] == "modify":
            osm_child.attrib.pop("action")
        if "lat" in osm_child.attrib:
            osm_child.attrib["lat"] = str(round(float(osm_child.attrib["lat"]), decimal_precision))
        if "lon" in osm_child.attrib:
            osm_child.attrib["lon"] = str(round(float(osm_child.attrib["lon"]), decimal_precision))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-s", "--sort-attributes", action="store_true", help="Sort attribues of LL2 maps"
    )
    parser.add_argument(
        "-i",
        "--ignore-minor-attributes",
        action="store_true",
        help="Ignore minor attribues of LL2 maps which does not change the map's behavior",
    )
    args = parser.parse_args()

    input_osm_file_name = "/tmp/static_centerline_generator/input/lanelet2_map.osm"
    output_osm_file_name = "/tmp/static_centerline_generator/output/lanelet2_map.osm"

    # load LL2 maps
    input_osm_tree = ET.parse(input_osm_file_name)
    input_osm_root = input_osm_tree.getroot()
    output_osm_tree = ET.parse(output_osm_file_name)
    output_osm_root = output_osm_tree.getroot()

    # sort attributes
    if args.sort_attributes:
        sort_attributes(output_osm_root)
        sort_attributes(input_osm_root)

    # ignore minor attributes
    if args.ignore_minor_attributes:
        remove_diff_to_ignore(input_osm_root)
        remove_diff_to_ignore(output_osm_root)

    # write LL2 maps
    output_dir_path = "/tmp/static_centerline_generator/tmp/"
    os.makedirs(output_dir_path + "input/", exist_ok=True)
    os.makedirs(output_dir_path + "output/", exist_ok=True)

    input_osm_tree.write(output_dir_path + "input/lanelet2_map.osm")
    output_osm_tree.write(output_dir_path + "output/lanelet2_map.osm")

    # show diff
    print("[INFO] Show diff of following LL2 maps")
    print(f"  {output_dir_path + 'input/lanelet2_map.osm'}")
    print(f"  {output_dir_path + 'output/lanelet2_map.osm'}")
    subprocess.run(
        [
            "diff",
            output_dir_path + "input/lanelet2_map.osm",
            output_dir_path + "output/lanelet2_map.osm",
            "--color",
        ]
    )
