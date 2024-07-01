# -*-coding: utf-8 -*-
"""
    @Author : panjq
    @E-mail : pan_jinquan@163.com
    @Date   : 2021-04-07 10:13:48
"""
import torch
from utils.torch_utils import select_device
from engine.inference import yolov5, onnx_detector


class YOLOv5ONNX(yolov5.YOLOv5):
    def __init__(self,
                 weights='yolov5s.pt',  # model.pt path(s)
                 imgsz=640,  # inference size (pixels)
                 conf_thres=0.25,  # confidence threshold
                 iou_thres=0.45,  # NMS IOU threshold
                 max_det=1000,  # maximum detections per image
                 classes=None,  # filter by class: --class 0, or --class 0 2 3
                 agnostic_nms=False,  # class-agnostic NMS
                 augment=False,  # augmented inference
                 half=False,  # use FP16 half-precision inference
                 visualize=False,  # visualize features
                 device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                 fix_inputs=True,
                 ):
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.visualize = visualize
        self.augment = augment
        self.max_det = max_det
        self.fix_inputs = fix_inputs
        # Initialize
        self.half = half
        self.device = select_device(device)
        self.half &= self.device.type != 'cpu'  # half precision only supported on CUDA
        # Load model
        self.stride = 32
        self.names = []
        # self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size
        if isinstance(imgsz, int):
            imgsz = (imgsz, imgsz)
        self.imgsz = imgsz
        self.model = onnx_detector.ONNXModel(weights)

    def inference(self, bgr_image):
        """
        :param bgr_image:
        """
        with torch.no_grad():
            input_image = self.preprocess(bgr_image)
            input_image = input_image.cpu().numpy()
            # pred = self.model(input_image, augment=self.augment, visualize=self.visualize)[0]
            pred = torch.tensor(self.model(input_image)[0])
            dets = self.postprocess(input_image, bgr_image.shape, pred)
        return dets


if __name__ == "__main__":
    opt = yolov5.parse_opt()
    # opt.weights = 'pretrained/yolov5s.onnx'
    opt.weights = 'pretrained/face_person.onnx'
    d = YOLOv5ONNX(weights=opt.weights,  # model.pt path(s)
                   imgsz=opt.imgsz,  # inference size (pixels)
                   conf_thres=opt.conf_thres,  # confidence threshold
                   iou_thres=opt.iou_thres,  # NMS IOU threshold
                   max_det=opt.max_det,  # maximum detections per image
                   classes=opt.classes,  # filter by class: --class 0, or --class 0 2 3
                   agnostic_nms=opt.agnostic_nms,  # class-agnostic NMS
                   augment=opt.augment,  # augmented inference
                   device=opt.device,  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                   )
    # d.detect_image_loader(opt.image_dir)
    d.detect_image_dir(opt.image_dir)
