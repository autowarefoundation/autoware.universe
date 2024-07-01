# -*-coding: utf-8 -*-
"""
    @Author : panjq
    @E-mail : pan_jinquan@163.com
    @Date   : 2022-09-14 14:25:31
    @Brief  :
"""

import os
import random
from tqdm import tqdm
from pybaseutils import file_utils, image_utils


def split_trainval_datset(image_dir, val_nums=200):
    """
    :param image_dir: 一类一个文件夹
    :param class_file: 如果含有class_file，会进行映射ID
    :param out_root:  保存格式[path/to/image,label]
    :param shuffle:
    :return:
    """
    image_list = file_utils.get_files_lists(image_dir)
    image_list = file_utils.get_sub_list(image_list, image_dir)
    random.seed(100)
    random.shuffle(image_list)
    val_list: list = image_list[0:val_nums]
    train_list: list = image_list[val_nums:]
    train_dir = file_utils.create_dir(image_dir, "train")
    val_dir = file_utils.create_dir(image_dir, "val")
    copy_move_files(image_dir, val_list, val_dir)
    copy_move_files(image_dir, train_list, train_dir)


def copy_move_files(image_dir, file_list, out_root):
    for image_id in file_list:
        src_path = os.path.join(image_dir, image_id)
        if not os.path.exists(src_path):
            print("not exists:{}".format(src_path))
            continue
        dst_path = os.path.join(out_root, image_id)
        # file_utils.copy_file(src_path, dst_path)
        file_utils.move_file(src_path, dst_path)


def HaGRID_Classification(image_dir):
    """
    将HaGRID转换为VOC和分类数据集
    :param image_dir: HaGRID数据集个根目录
    :param vis: 是否可视化效果
    :return:
    """
    sub_list = file_utils.get_sub_paths(image_dir)
    for sub in sub_list:
        image_dir_ = os.path.join(image_dir, sub, "Classification")
        print("process:{}".format(image_dir_))
        split_trainval_datset(image_dir_)


if __name__ == "__main__":
    image_dir = "/home/dm/nasdata/dataset/csdn/gesture/HaGRID111/"
    # save_file_list_labels(image_dir)
    HaGRID_Classification(image_dir)
