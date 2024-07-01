# -*-coding: utf-8 -*-
"""
    @Author : panjq
    @E-mail : pan_jinquan@163.com
    @Date   : 2022-09-14 14:25:31
    @Brief  :
"""

import os
import numpy as np
import cv2
from tqdm import tqdm
from pybaseutils import image_utils, file_utils
from pybaseutils.maker import maker_voc


def copy_HaGRID_images(data_root, out_dir=None, phase="val", max_num=4, vis=True):
    """
    将HaGRID转换为VOC和分类数据集
    :param data_root: HaGRID数据集个根目录
    :param vis: 是否可视化效果
    :return:
    """
    sub_list = file_utils.get_sub_paths(data_root)
    for sub in sub_list:
        file = os.path.join(data_root, sub, "{}.txt".format(phase))
        image_ids = file_utils.read_data(file, split=None)
        nums = min(max_num, len(image_ids)) if max_num else len(image_ids)
        image_ids = image_ids[0:nums]
        print("process:{},nums:{}".format(file, len(image_ids)))
        for image_id in tqdm(image_ids):
            src_file = os.path.join(data_root, sub, "JPEGImages", image_id)
            if not os.path.exists(src_file):
                print("not exists:{}".format(src_file))
                continue
            if out_dir:
                dst_dir = file_utils.create_dir(out_dir, sub, "JPEGImages")
                file_utils.copy_file_to_dir(src_file, dst_dir)


if __name__ == "__main__":
    data_root = "/home/dm/nasdata/dataset/csdn/gesture/HaGRID/trainval"
    out_dir = "/home/dm/nasdata/Detector/YOLO/yolov5/data/HaGRID-test"
    copy_HaGRID_images(data_root, out_dir=out_dir, vis=True)
