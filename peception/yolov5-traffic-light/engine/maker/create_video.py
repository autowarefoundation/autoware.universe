# -*-coding: utf-8 -*-
"""
    @Author : panjq
    @E-mail : pan_jinquan@163.com
    @Date   : 2022-09-15 16:45:52
    @Brief  :
"""
import random
from pybaseutils import image_utils, file_utils, video_utils


def frame2video(fps=25):
    image_dir = "/home/dm/nasdata/Detector/YOLO/yolov5/data/helmet-test"
    video_file = "/home/dm/nasdata/Detector/YOLO/yolov5/data/helmet-test.mp4"
    size = (640, 640)
    image_list = file_utils.get_files_list(image_dir, postfix=["*.png", "*.jpg"])
    image_list = image_list * fps
    image_list = sorted(image_list)
    video_utils.frames2video(image_list, video_file=video_file, size=size, fps=25)


if __name__ == "__main__":
    frame2video()
