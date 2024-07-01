# YOLOv5 dataset utils and dataloaders

import glob
import hashlib
import json
import logging
import os
import random
import shutil
import time
from itertools import repeat
from multiprocessing.pool import ThreadPool, Pool
from pathlib import Path
from threading import Thread

import cv2
import numpy as np
import torch
import torch.nn.functional as F
import yaml
from PIL import Image, ExifTags
from torch.utils.data import Dataset
from utils.augmentations import Albumentations, augment_hsv, copy_paste, letterbox, mixup, random_perspective
from utils.general import check_requirements, check_file, check_dataset, xywh2xyxy, xywhn2xyxy, xyxy2xywhn, \
    xyn2xy, segments2boxes, clean_str
from utils.torch_utils import torch_distributed_zero_first
from utils import file_utils, image_utils, parser_voc, parser_word
from utils import datasets
from tqdm import tqdm

# Parameters
HELP_URL = 'https://github.com/ultralytics/yolov5/wiki/Train-Custom-Data'
IMG_FORMATS = ['bmp', 'jpg', 'jpeg', 'png', 'tif', 'tiff', 'dng', 'webp', 'mpo']  # acceptable image suffixes
VID_FORMATS = ['mov', 'avi', 'mp4', 'mpg', 'mpeg', 'm4v', 'wmv', 'mkv']  # acceptable video suffixes
NUM_THREADS = min(8, os.cpu_count())  # number of multiprocessing threads

# Get orientation exif tag
for orientation in ExifTags.TAGS.keys():
    if ExifTags.TAGS[orientation] == 'Orientation':
        break


def get_hash(paths):
    # Returns a single hash value of a list of paths (files or dirs)
    size = sum(os.path.getsize(p) for p in paths if os.path.exists(p))  # sizes
    h = hashlib.md5(str(size).encode())  # hash sizes
    h.update(''.join(paths).encode())  # hash paths
    return h.hexdigest()  # return hash


def exif_size(img):
    # Returns exif-corrected PIL size
    s = img.size  # (width, height)
    try:
        rotation = dict(img._getexif().items())[orientation]
        if rotation == 6:  # rotation 270
            s = (s[1], s[0])
        elif rotation == 8:  # rotation 90
            s = (s[1], s[0])
    except:
        pass

    return s


def exif_transpose(image):
    """
    Transpose a PIL image accordingly if it has an EXIF Orientation tag.
    From https://github.com/python-pillow/Pillow/blob/master/src/PIL/ImageOps.py

    :param image: The image to transpose.
    :return: An image.
    """
    exif = image.getexif()
    orientation = exif.get(0x0112, 1)  # default 1
    if orientation > 1:
        method = {2: Image.FLIP_LEFT_RIGHT,
                  3: Image.ROTATE_180,
                  4: Image.FLIP_TOP_BOTTOM,
                  5: Image.TRANSPOSE,
                  6: Image.ROTATE_270,
                  7: Image.TRANSVERSE,
                  8: Image.ROTATE_90,
                  }.get(orientation)
        if method is not None:
            image = image.transpose(method)
            del exif[0x0112]
            image.info["exif"] = exif.tobytes()
    return image


def img2label_paths(img_paths):
    # Define label paths as a function of image paths
    sa, sb = os.sep + 'images' + os.sep, os.sep + 'labels' + os.sep  # /images/, /labels/ substrings
    return [sb.join(x.rsplit(sa, 1)).rsplit('.', 1)[0] + '.txt' for x in img_paths]


class LoadVOCImagesAndLabels(datasets.LoadImagesAndLabels):  # for training/testing
    def __init__(self, path, img_size=640, batch_size=16, augment=False, hyp=None, rect=False, image_weights=False,
                 cache_images=False, single_cls=False, stride=32, pad=0.0, prefix='', names=None):
        if isinstance(path, str):
            path = [path]
        dataset_list = []
        for file in path:
            print("load data:{}".format(file))
            voc = parser_voc.VOCDataset(filename=file,
                                        data_root=None,
                                        anno_dir=None,
                                        image_dir=None,
                                        class_name=names,
                                        transform=None,
                                        use_rgb=False,
                                        shuffle=False,
                                        check=False)
            dataset_list.append(voc)
        self.voc = parser_voc.ConcatDataset(dataset_list, shuffle=True)
        self.img_size = img_size
        self.augment = augment
        self.hyp = hyp
        self.image_weights = image_weights
        self.rect = False if image_weights else rect
        self.mosaic = self.augment and not self.rect  # load 4 images at a time into a mosaic (only during training)
        self.mosaic_border = [-img_size // 2, -img_size // 2]
        self.stride = stride
        self.albumentations = Albumentations() if augment else None
        self.img_files = self.voc.image_id
        cache, exists = self.cache_labels(), False  # cache
        # Read cache
        [cache.pop(k) for k in ('hash', 'version', 'msgs', 'results')]  # remove items
        image_files, labels, shapes, self.segments = zip(*cache.values())
        self.labels = list(labels)
        self.shapes = np.array(shapes, dtype=np.float64)
        # self.img_files = list(cache.keys())  # update
        self.img_files = image_files  # update
        if single_cls:
            for x in self.labels:
                x[:, 0] = 0

        n = len(shapes)  # number of images
        bi = np.floor(np.arange(n) / batch_size).astype(np.int)  # batch index
        nb = bi[-1] + 1  # number of batches
        self.batch = bi  # batch index of image
        self.n = n
        self.indices = range(n)

        # Rectangular Training
        if self.rect:
            # Sort by aspect ratio
            s = self.shapes  # wh
            ar = s[:, 1] / s[:, 0]  # aspect ratio
            irect = ar.argsort()
            self.img_files = [self.img_files[i] for i in irect]
            self.labels = [self.labels[i] for i in irect]
            self.shapes = s[irect]  # wh
            ar = ar[irect]

            # Set training image shapes
            shapes = [[1, 1]] * nb
            for i in range(nb):
                ari = ar[bi == i]
                mini, maxi = ari.min(), ari.max()
                if maxi < 1:
                    shapes[i] = [maxi, 1]
                elif mini > 1:
                    shapes[i] = [1, 1 / mini]

            self.batch_shapes = np.ceil(np.array(shapes) * img_size / stride + pad).astype(np.int) * stride

        # Cache images into memory for faster training (WARNING: large datasets may exceed system RAM)
        self.imgs, self.img_npy = [None] * n, [None] * n
        if cache_images:
            if cache_images == 'disk':
                self.im_cache_dir = Path(Path(self.img_files[0]).parent.as_posix() + '_npy')
                self.img_npy = [self.im_cache_dir / Path(f).with_suffix('.npy').name for f in self.img_files]
                self.im_cache_dir.mkdir(parents=True, exist_ok=True)
            gb = 0  # Gigabytes of cached images
            self.img_hw0, self.img_hw = [None] * n, [None] * n
            results = ThreadPool(NUM_THREADS).imap(lambda x: datasets.load_image(*x), zip(repeat(self), range(n)))
            pbar = tqdm(enumerate(results), total=n)
            for i, x in pbar:
                if cache_images == 'disk':
                    if not self.img_npy[i].exists():
                        np.save(self.img_npy[i].as_posix(), x[0])
                    gb += self.img_npy[i].stat().st_size
                else:
                    self.imgs[i], self.img_hw0[i], self.img_hw[i] = x  # im, hw_orig, hw_resized = load_image(self, i)
                    gb += self.imgs[i].nbytes
                pbar.desc = f'{prefix}Caching images ({gb / 1E9:.1f}GB {cache_images})'
            pbar.close()

    def cache_labels(self):
        # Cache dataset labels, check images and read shapes
        x = {}  # dict
        print("load image annotation data")
        for index in tqdm(range(len(self.img_files))):
            # print(index)
            # index = 74
            image_file, ann_file = self.voc.get_image_anno_file(index)
            objects = self.voc.get_annotation(ann_file)
            bboxes, labels, is_difficult = objects["bboxes"], objects["labels"], objects["is_difficult"]
            if len(labels) == 0: continue
            bboxes = self.xyxy2cxcywh(bboxes)
            height, width = objects["height"], objects["width"]
            bboxes = bboxes / (width, height, width, height)
            labels = np.concatenate([labels.reshape(-1, 1), bboxes], axis=1)
            # x[image_file] = [labels, (width, height), []]
            x[str(index) + image_file] = [image_file, labels, (width, height), []]
            # image = self.voc.read_image(image_file)
            # self.show_cxcywh(image, labels)
        x['hash'] = ""
        # found, missing, empty, corrupted, total
        x['results'] = (len(self.img_files), 0, 0, 0, len(self.img_files))
        x['msgs'] = ""  # warnings
        x['version'] = 0.4  # cache version
        return x

    def xyxy2cxcywh(self, xyxy):
        cxcywh = np.zeros_like(xyxy)
        cxcywh[:, 0] = (xyxy[:, 2] + xyxy[:, 0]) / 2  # cx
        cxcywh[:, 1] = (xyxy[:, 3] + xyxy[:, 1]) / 2  # cy
        cxcywh[:, 2] = (xyxy[:, 2] - xyxy[:, 0])  # w
        cxcywh[:, 3] = (xyxy[:, 3] - xyxy[:, 1])  # h
        return cxcywh

    def __getitem__(self, index):
        """
        返回模型输入的图片是RGB格式
        """
        index = self.indices[index]  # linear, shuffled, or image_weights
        # index = self.indices[0]  # linear, shuffled, or image_weights

        hyp = self.hyp
        mosaic = self.mosaic and random.random() < hyp['mosaic']
        if mosaic:
            # Load mosaic
            img, labels = datasets.load_mosaic(self, index)
            shapes = None

            # MixUp augmentation
            if random.random() < hyp['mixup']:
                img, labels = mixup(img, labels, *datasets.load_mosaic(self, random.randint(0, self.n - 1)))

        else:
            # Load image
            img, (h0, w0), (h, w) = datasets.load_image(self, index)

            # Letterbox
            shape = self.batch_shapes[self.batch[index]] if self.rect else self.img_size  # final letterboxed shape
            img, ratio, pad = letterbox(img, shape, auto=False, scaleup=self.augment)
            shapes = (h0, w0), ((h / h0, w / w0), pad)  # for COCO mAP rescaling

            labels = self.labels[index].copy()
            if labels.size:  # normalized xywh to pixel xyxy format
                labels[:, 1:] = xywhn2xyxy(labels[:, 1:], ratio[0] * w, ratio[1] * h, padw=pad[0], padh=pad[1])

            if self.augment:
                img, labels = random_perspective(img, labels,
                                                 degrees=hyp['degrees'],
                                                 translate=hyp['translate'],
                                                 scale=hyp['scale'],
                                                 shear=hyp['shear'],
                                                 perspective=hyp['perspective'])

        nl = len(labels)  # number of labels
        if nl:
            labels[:, 1:5] = xyxy2xywhn(labels[:, 1:5], w=img.shape[1], h=img.shape[0], clip=True, eps=1E-3)

        if self.augment:
            # Albumentations
            img, labels = self.albumentations(img, labels)

            # HSV color-space
            augment_hsv(img, hgain=hyp['hsv_h'], sgain=hyp['hsv_s'], vgain=hyp['hsv_v'])

            # Flip up-down
            if random.random() < hyp['flipud']:
                img = np.flipud(img)
                if nl:
                    labels[:, 2] = 1 - labels[:, 2]

            # Flip left-right
            if random.random() < hyp['fliplr']:
                img = np.fliplr(img)
                if nl:
                    labels[:, 1] = 1 - labels[:, 1]

            # Cutouts
            # labels = cutout(img, labels, p=0.5)
        # self.show_cxcywh(img, labels)
        labels_out = torch.zeros((nl, 6))
        if nl:
            labels_out[:, 1:] = torch.from_numpy(labels)

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)

        return torch.from_numpy(img), labels_out, self.img_files[index], shapes

    def show_cxcywh(self, img, labels):
        height, width, d = img.shape
        rects = []
        classes = []
        for item in labels:
            c, cx, cy, w, h = item
            rect = [(cx - w / 2) * width, (cy - h / 2) * height, w * width, h * height]
            rects.append(rect)
            classes.append(c)
        image_utils.show_image_rects_text("image", img, rects, classes)


def show_boxes_image(image, labels, normal=False, transpose=False):
    """
    :param image:
    :param targets_t:
                bboxes = targets[idx][:, :4].data
                keypoints = targets[idx][:, 4:14].data
                labels = targets[idx][:, -1].data
    :return:
    """
    import numpy as np
    from pybaseutils import image_utils, coords_utils
    image = np.asarray(image)
    labels = np.asarray(labels)  # 0, c, cx, cy, w, h
    bboxes = labels[:, 2:6]
    labels = labels[:, 1:2]
    print("image:{}".format(image.shape))
    print("bboxes:{}".format(bboxes))
    print("labels:{}".format(labels))
    if transpose:
        image = image_utils.untranspose(image)
    h, w, _ = image.shape
    landms_scale = np.asarray([w, h] * 5)
    bboxes_scale = np.asarray([w, h] * 2)
    bboxes = coords_utils.cxcywh2xyxy(bboxes, width=w, height=h, normalized=normal)
    # image = image_processing.untranspose(image)
    # image = image_processing.convert_color_space(image, colorSpace="RGB")
    image = image_utils.draw_image_bboxes_text(image, bboxes, labels, thickness=1, color=(0, 255, 0))
    image_utils.cv_show_image("image", image, delay=0, use_rgb=True)
    print("===" * 10)


if __name__ == "__main__":
    """
    scale: 对图片进行缩放0-1.0
    """
    imgsz = 640
    batch_size = 16
    augment = True
    hyp = {"lr0": 0.01, "lrf": 0.2, "momentum": 0.937, "weight_decay": 0.0005, "warmup_epochs": 3.0,
           "warmup_momentum": 0.8, "warmup_bias_lr": 0.1, "box": 0.05, "cls": 0.5, "cls_pw": 1.0, 'obj': 1.0,
           "obj_pw": 1.0, 'iou_t': 0.2, 'anchor_t': 4.0, "fl_gamma": 0.0, 'hsv_h': 0.015, 'hsv_s': 0.7,
           "hsv_v": 0.4, "degrees": 5.0, "translate": 0.1, "scale": 0.2, "shear": 0.0, "perspective": 0.0,
           "flipud": 0.0, "fliplr": 0.5, "mosaic": 0.0, "mixup": 0.0, "copy_paste": 0.0}
    rect = False
    cache = None
    single_cls = False
    stride = 32
    pad = 0.0
    image_weights = False
    # names = ['unique']
    # names = {"unique": 0}
    names = "/home/dm/cv/panjinquan/dataset/csdn/traffic light/Traffic-Lights-Dataset-Domestic/class_name.txt"
    path = '/home/dm/cv/panjinquan/dataset/csdn/traffic light/Traffic-Lights-Dataset-Domestic/val.txt'

    # names = "/home/dm/nasdata/dataset/csdn/helmet/class_name.txt"
    # path = "/home/dm/nasdata/dataset/csdn/helmet/test.txt"
    prefix = "prefix"
    dataset = LoadVOCImagesAndLabels(path,
                                     imgsz,
                                     batch_size,
                                     augment=augment,  # augment images
                                     hyp=hyp,  # augmentation hyperparameters
                                     rect=rect,  # rectangular training
                                     cache_images=cache,
                                     single_cls=single_cls,
                                     stride=int(stride),
                                     pad=pad,
                                     image_weights=image_weights,
                                     prefix=prefix,
                                     names=names)

    print("have total sample:{}".format(len(dataset)))
    for data in dataset:
        image, labels, file, shape = data
        # labels is:0, c, cx, cy, w, h
        show_boxes_image(image, labels, normal=True, transpose=True)
