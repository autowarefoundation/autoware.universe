import torch
import numpy as np
import cv2
from utils.datasets import letterbox
from utils.general import non_max_suppression, scale_coords
from utils.plots import Annotator
from dynamikontrol import Module

module = Module()
module.base_led.mix((100, 0, 0))

img_size = 640
conf_thres = 0.5  # confidence threshold
iou_thres = 0.45  # NMS IOU threshold
max_det = 1000  # maximum detections per image
classes = None  # filter by class
agnostic_nms = False  # class-agnostic NMS

device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
ckpt = torch.load('runs/train/exp4/weights/best.pt', map_location=device)
model = ckpt['ema' if ckpt.get('ema') else 'model'].float().fuse().eval()
class_names = ['횡단보도', '빨간불', '초록불'] # model.names
stride = int(model.stride.max())
colors = ((50, 50, 50), (0, 0, 255), (0, 255, 0)) # (gray, red, green)

cap = cv2.VideoCapture('data/sample.mp4')

while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        break

    # preprocess
    img_input = letterbox(img, img_size, stride=stride)[0]
    img_input = img_input.transpose((2, 0, 1))[::-1]
    img_input = np.ascontiguousarray(img_input)
    img_input = torch.from_numpy(img_input).to(device)
    img_input = img_input.float()
    img_input /= 255.
    img_input = img_input.unsqueeze(0)

    # inference
    pred = model(img_input, augment=False, visualize=False)[0]

    # postprocess
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)[0]
    pred = pred.cpu().numpy()
    pred[:, :4] = scale_coords(img_input.shape[2:], pred[:, :4], img.shape).round()

    annotator = Annotator(img.copy(), line_width=3, example=str(class_names), font='data/malgun.ttf')

    for p in pred:
        class_name = class_names[int(p[5])]

        if class_name == '초록불':
            module.base_led.mix((0, 100, 0))
            module.motor.angle(90)
        elif class_name == '빨간불':
            module.base_led.mix((100, 0, 0))
            module.motor.angle(0)

        annotator.box_label(p[:4], '%s %d' % (class_name, float(p[4]) * 100), color=colors[int(p[5])])

    result_img = annotator.result()

    cv2.imshow('result', result_img)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()

module.base_led.mix((100, 100, 100))
module.motor.angle(0)
module.disconnect()
