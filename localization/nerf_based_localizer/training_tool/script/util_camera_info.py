import yaml
import numpy as np


def camera_info_to_dict(camera_info):
    return {
        "header": {
            "stamp": {"sec": camera_info.header.stamp.sec, "nanosec": camera_info.header.stamp.nanosec},
            "frame_id": camera_info.header.frame_id
        },
        "height": camera_info.height,
        "width": camera_info.width,
        "distortion_model": camera_info.distortion_model,
        "D": camera_info.d.tolist(),
        "K": camera_info.k.tolist(),
        "R": camera_info.r.tolist(),
        "P": camera_info.p.tolist(),
        "binning_x": camera_info.binning_x,
        "binning_y": camera_info.binning_y,
        "roi": {
            "x_offset": camera_info.roi.x_offset,
            "y_offset": camera_info.roi.y_offset,
            "height": camera_info.roi.height,
            "width": camera_info.roi.width,
            "do_rectify": camera_info.roi.do_rectify
        }
    }


def save_camera_info_to_yaml(camera_info, filename):
    camera_info_dict = camera_info_to_dict(camera_info)
    with open(filename, "w") as outfile:
        yaml.dump(camera_info_dict, outfile)


def load_camera_info_from_yaml(filename):
    with open(filename, "r") as input_file:
        camera_info_dict = yaml.safe_load(input_file)
        camera_info_dict["D"] = np.array(camera_info_dict["D"])
        camera_info_dict["K"] = np.array(camera_info_dict["K"]).reshape((3, 3))
        camera_info_dict["R"] = np.array(camera_info_dict["R"]).reshape((3, 3))
        camera_info_dict["P"] = np.array(camera_info_dict["P"]).reshape((3, 4))
        return camera_info_dict
