"""Run inference with a YOLOv5 model on images, videos, directories, streams

Usage:
    $ python path/to/detect.py --source path/to/img.jpg --weights yolov5s.pt --img 640
"""

import argparse
import cv2
import torch
import numpy as np
from models.experimental import attempt_load, attempt_load_model
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, non_max_suppression, apply_classifier, scale_coords
from utils.plots import colors, plot_one_box
from utils.torch_utils import select_device, load_classifier
from utils import file_utils, image_utils
from pybaseutils import batch_utils
from utils.augmentations import letterbox


class YOLOv5(object):
    def __init__(self,
                 weights='yolov5s.pt',  # model.pt path(s)
                 imgsz=640,  # inference size (pixels)
                 conf_thres=0.25,  # confidence threshold
                 iou_thres=0.45,  # NMS IOU threshold
                 max_det=1000,  # maximum detections per image
                 class_name=None,  # filter by class: --class 0, or --class 0 2 3
                 classes=None,  # filter by class: --class 0, or --class 0 2 3
                 agnostic_nms=False,  # class-agnostic NMS
                 augment=False,  # augmented inference
                 half=False,  # use FP16 half-precision inference
                 visualize=False,  # visualize features
                 batch_size=4,
                 device='cuda:0',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                 fix_inputs=True,
                 ):
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.class_name = class_name
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.visualize = visualize
        self.augment = augment
        self.max_det = max_det
        self.batch_size = batch_size
        self.fix_inputs = fix_inputs
        # self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size
        if isinstance(imgsz, int):
            imgsz = (imgsz, imgsz)
        self.imgsz = imgsz
        # Initialize
        self.half = half
        # self.device = select_device(device)
        self.device = torch.device(device)
        self.half &= self.device.type != 'cpu'  # half precision only supported on CUDA
        # Load model
        self.stride, self.names = 64, [f'class{i}' for i in range(1000)]  # assign defaults
        self.model = attempt_load(weights, map_location=self.device)  # load FP32 model
        # self.model = attempt_load_model(weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        # get class names
        if self.class_name:
            self.names = self.class_name
        else:
            self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        if isinstance(self.names, dict):
            self.names = {v: k for k, v in self.names.items()}
        if self.half:
            self.model.half()  # to FP16
        print("class_name:{}".format(self.names))
        # torch_tools.summary_model(self.model, batch_size=1, input_size= self.imgsz, device="cpu")

    def preprocess(self, images):
        if isinstance(images, np.ndarray): images = [images]
        image_tensors = []
        image_shapes = []
        for img in images:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # Padded resize
            if self.fix_inputs:
                input_image = image_utils.image_boxes_resize_padding(img, self.imgsz, color=(114, 114, 114))
            else:
                # input_image = letterbox(image, self.imgsz, stride=self.stride)[0]
                input_image = letterbox(img, self.imgsz, stride=self.stride, auto=False, scaleFill=False)[0]
            # image_utils.cv_show_image("input_image", input_image)
            # Convert
            input_image = input_image.transpose(2, 0, 1)  # HWC->CHW

            input_image = torch.from_numpy(input_image).to(self.device)
            input_image = input_image.half() if self.half else input_image.float()  # uint8 to fp16/32
            input_image /= 255.0  # 0 - 255 to 0.0 - 1.0
            # input_image = input_image.  # expand for batch dim
            input_image = torch.unsqueeze(input_image, dim=0)
            image_tensors.append(input_image)
            image_shapes.append(img.shape)
        image_tensors = torch.cat(image_tensors)
        return image_tensors, image_shapes

    def postprocess(self, image_tensors, image_shapes, pred):
        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes,
                                   self.agnostic_nms, max_det=self.max_det)
        # Process predictions
        dets = []
        for i in range(len(pred)):
            det = pred[i].cpu().numpy()
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(image_tensors.shape[2:], det[:, :4], image_shapes[i]).round()
                # h, w = image_shapes[i][:2]
                # det[:, :4] = image_utils.image_boxes_resize_padding_inverse((w, h), self.imgsz, det[:, :4]).round()
            dets.append(det)
        return dets

    def inference(self, bgr_image):
        """
        训练图片是RGB格式，输入图像要是BGR格式，在preprocess中转换为RGB格式
        :param image: 图像或者图像列表,BGR格式
        :param vis: 是否可视化显示检测结果
        :return: 返回检测结果[[List]], each bounding box is in [x1,y1,x2,y2,conf,cls] format.
        """
        if isinstance(bgr_image, np.ndarray): bgr_image = [bgr_image]
        batch_dets = []
        for batch in batch_utils.get_batch_sample(bgr_image, batch_size=self.batch_size):
            with torch.no_grad():
                image_tensors, image_shapes = self.preprocess(batch)
                pred = self.model(image_tensors, augment=self.augment, visualize=self.visualize)[0]
                dets = self.postprocess(image_tensors, image_shapes, pred)
                batch_dets += dets
        return batch_dets

    def detect_image_loader(self, image_dir):
        # Dataloader
        dataset = LoadImages(image_dir, img_size=self.imgsz, stride=self.stride)
        # Run inference
        for path, input_image, image, vid_cap in dataset:
            # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            dets = self.inference(image)
            self.draw_result(image, dets)

    def detect_image_dir(self, image_dir):
        # Dataloader
        dataset = file_utils.get_files_lists(image_dir)
        # Run inference
        for path in dataset:
            image = cv2.imread(path)  # BGR
            # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            dets = self.inference(image)
            self.draw_result(image, dets)

    def draw_result(self, image, dets, waitKey=0):
        # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # (xmin,ymin,xmax,ymax,conf, cls)
        boxes = dets[:, 0:4]
        conf = dets[:, 4:5]
        cls = dets[:, 5]
        labels = [int(c) for c in cls]
        if self.names:
            labels = [self.names[int(c)] for c in cls]
        image_utils.draw_image_detection_bboxes(image, boxes, conf, labels)
        # for *box, conf, cls in reversed(dets):
        #     c = int(cls)  # integer class
        #     label = "{}{:.2f}".format(self.names[c], conf)
        #     plot_one_box(box, image, label=label, color=colors(c, True), line_thickness=2)
        cv2.imshow("image", image)
        cv2.waitKey(waitKey)  # 1 millisecond
        return image


def parse_opt():
    # weights = 'pretrained/face_person.pt'
    # weights = 'pretrained/yolov5s.pt'
    image_dir = '/data/images'
    image_dir = '/home/dm/nasdata/dataset/MCLZ/JPEGImages'
    # image_dir = 'data/test_image'
    weights = "/home/dm/data3/FaceDetector/YOLO/yolov5/runs/mclz/exp2/weights/best.pt"
    class_name = ["face", "person"]
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=weights, help='model.pt')
    parser.add_argument('--image_dir', type=str, default=image_dir, help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--imgsz', '--img', '--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--class_name', nargs='+', type=list, default=class_name)
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    opt = parser.parse_args()
    return opt


if __name__ == "__main__":
    opt = parse_opt()
    d = YOLOv5(weights=opt.weights,  # model.pt path(s)
               imgsz=opt.imgsz,  # inference size (pixels)
               conf_thres=opt.conf_thres,  # confidence threshold
               iou_thres=opt.iou_thres,  # NMS IOU threshold
               max_det=opt.max_det,  # maximum detections per image
               class_name=opt.class_name,  # filter by class: --class 0, or --class 0 2 3
               classes=opt.classes,  # filter by class: --class 0, or --class 0 2 3
               agnostic_nms=opt.agnostic_nms,  # class-agnostic NMS
               augment=opt.augment,  # augmented inference
               device=opt.device,  # cuda device, i.e. 0 or 0,1,2,3 or cpu
               )
    # d.detect_image_loader(opt.image_dir)
    d.detect_image_dir(opt.image_dir)
