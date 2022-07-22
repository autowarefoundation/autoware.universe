import os
import time

from perception_benchmark_tool.benchmark_tools.datasets.waymo_dataset.waymo_decode_data import (
    extract_dataset_from_tfrecord,
)


class WaymoDataset:
    def __init__(self, segment_path):

        self.lidars_tf = {}
        self.cameras_tf = {}

        self.segment_path = os.path.join(segment_path)
        dataset_extract_start = time.time()
        (
            self.dataset_scene_by_scene,
            self.lidars_tf,
            self.cameras_tf,
        ) = extract_dataset_from_tfrecord(self.segment_path)
        dataset_extract_end = time.time()
        print(
            "Dataset prepared, in " + str(dataset_extract_end - dataset_extract_start) + " seconds."
        )
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
