# -*-coding: utf-8 -*-
"""
    @Author : panjq
    @E-mail : pan_jinquan@163.com
    @Date   : 2022-06-09 10:31:41
    @Brief  :
"""
import numpy as np
import utils.autoanchor as autoAC


def get_default_anchor(input_size=416):
    """"""
    size = 640
    coco_anchor = [[10, 13, 16, 30, 33, 23],
                   [30, 61, 62, 45, 59, 119],
                   [116, 90, 156, 198, 373, 326]]
    scale = input_size / size
    anchor = np.asarray(coco_anchor) * scale
    return anchor


if __name__ == "__main__":
    """
    YOLOv5s-640原始Anchor(通用):
      - [10,13, 16,30, 33,23]  # P3/8
      - [30,61, 62,45, 59,119]  # P4/16
      - [116,90, 156,198, 373,326]  # P5/32
      
      [[ 2  5  3  8  4 12]
     [ 6 16 18  8  8 19]
     [10 27 14 38 26 63]]
    """
    data = '/home/dm/nasdata/Detector/YOLO/yolov5/engine/configs/voc_local.yaml'
    anchors = autoAC.kmean_anchors(data, n=9, img_size=640, thr=4.0, gen=1000, verbose=False)
    # anchors = get_default_anchor()
    anchors = np.asarray(np.round(anchors), dtype=np.int32)
    print(anchors)
    anchors = anchors.reshape(3, -1)
    print(anchors)
