# -*-coding: utf-8 -*-
"""
    @Author : panjq
    @E-mail : pan_jinquan@163.com
    @Date   : 2022-09-15 16:45:52
    @Brief  :
"""
import random
from pybaseutils import image_utils, file_utils, video_utils


def frame2gif():
    image_dir = "/home/dm/nasdata/Detector/YOLO/yolov5/runs/helmet-result"
    gif_file = "/home/dm/nasdata/Detector/YOLO/yolov5/runs/helmet-result/helmet.gif"
    image_list = file_utils.get_images_list(image_dir)
    image_utils.image_file_list2gif(image_list, gif_file=gif_file,size=(416, 416), fps=1, loop=0, use_pil=False)


def video2gif():
    video_file = "/home/dm/桌面/image/HnVideoEditor_2022_10_16_110231782.mp4"

    def target_task(frame):
        frame = image_utils.resize_image(frame, size=(320, None))
        return frame

    video_utils.video2gif(video_file, gif_file=None, func=target_task, interval=5, use_pil=False, vis=True)


if __name__ == "__main__":
    # frame2gif()
    video2gif()
