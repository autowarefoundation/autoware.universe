# Copyright 2018 Autoware Foundation. All rights reserved.
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

import os

from perception_benchmark_tool.benchmark_tools.datasets.waymo_dataset.waymo_decode_data import (
    extract_dataset_from_tfrecord,
)


class WaymoDataset:
    def __init__(self, segment_path):

        self.lidars_tf = {}
        self.cameras_tf = {}

        self.segment_path = os.path.join(segment_path)

        (
            self.dataset_scene_by_scene,
            self.lidars_tf,
            self.cameras_tf,
        ) = extract_dataset_from_tfrecord(self.segment_path)

        self.frame_counter = 0

    def get_lidars_static_tf(self):
        return self.lidars_tf

    def get_cameras_static_tf(self):
        return self.cameras_tf

    def get_scene_from_dataset(self):
        self.frame_counter += 1
        return self.dataset_scene_by_scene[self.frame_counter - 1]

    def is_finished(self):
        if self.frame_counter < len(self.dataset_scene_by_scene):
            return False
        else:
            return True

    def get_all_dataset(self):
        return self.dataset_scene_by_scene
