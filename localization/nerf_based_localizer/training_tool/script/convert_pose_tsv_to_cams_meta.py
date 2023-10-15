import argparse
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation
import os
import yaml


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("target_dir", type=str)
    return parser.parse_args()


def load_camera_info_from_yaml(filename):
    with open(filename, "r") as input_file:
        camera_info_dict = yaml.safe_load(input_file)
        camera_info_dict["D"] = np.array(camera_info_dict["D"])
        camera_info_dict["K"] = np.array(camera_info_dict["K"]).reshape((3, 3))
        camera_info_dict["R"] = np.array(camera_info_dict["R"]).reshape((3, 3))
        camera_info_dict["P"] = np.array(camera_info_dict["P"]).reshape((3, 4))
        return camera_info_dict


AXIS_CONVERT_MAT_W2N = np.array(
    [[+1,  0,  0,  0],
     [ 0, -1,  0,  0],
     [ 0,  0, -1,  0],
     [ 0,  0,  0, +1]], dtype=np.float64
)


def generate_cams_meta(target_dir: str) -> None:
    df_pose = pd.read_csv(f"{target_dir}/pose.tsv", sep="\t", index_col=0)
    n = len(df_pose)
    pose_xyz = df_pose[['x', 'y', 'z']].values
    pose_quat = df_pose[['qx', 'qy', 'qz', 'qw']].values
    rotation_mat = Rotation.from_quat(pose_quat).as_matrix()
    mat = np.tile(np.eye(4), (n, 1, 1))
    mat[:, 0:3, 0:3] = rotation_mat
    mat[:, 0:3, 3:4] = pose_xyz.reshape((n, 3, 1))

    # convert axis
    mat = AXIS_CONVERT_MAT_W2N @ mat @ AXIS_CONVERT_MAT_W2N.T
    mat = mat[:, 0:3, :]
    mat = mat.reshape((n, 12))

    # save camera meta
    camera_info = load_camera_info_from_yaml(f"{target_dir}/camera_info.yaml")
    k = camera_info["K"]
    camera_param = np.tile(k, (n, 1, 1))
    camera_param = camera_param.reshape((n, 9))

    dist_param = camera_info["D"][0:4]
    dist_param = np.tile(dist_param, (n, 1))

    data = np.concatenate([mat, camera_param, dist_param], axis=1)

    # save as tsv
    df = pd.DataFrame(data, columns=[
        # mat
        "R00", "R01", "R02", "tx",
        "R10", "R11", "R12", "ty",
        "R20", "R21", "R22", "tz",
        # camera_param
        "p00", "p01", "p02",
        "p10", "p11", "p12",
        "p20", "p21", "p22",
        # dist_param
        "k1", "k2", "p1", "p2"
    ])
    df.to_csv(os.path.join(target_dir, "cams_meta.tsv"), sep="\t", index=False)


if __name__ == "__main__":
    args = parse_args()
    target_dir = args.target_dir
    generate_cams_meta(target_dir)
