import torch
import numpy as np
import cv2
from utils.datasets import letterbox
from utils.general import non_max_suppression, scale_coords
from utils.plots import Annotator

# 횡단보도,신호등 모델
MODEL_PATH = 'runs/train/exp4/weights/best.pt'

img_size = 640
conf_thres = 0.5  # confidence threshold
iou_thres = 0.45  # NMS IOU threshold
max_det = 1000  # maximum detections per image
classes = None  # filter by class
agnostic_nms = False  # class-agnostic NMS

device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
ckpt = torch.load(MODEL_PATH, map_location=device)
model = ckpt['ema' if ckpt.get('ema') else 'model'].float().fuse().eval()
class_names = ['횡단보도', '빨간불', '초록불'] # model.names
stride = int(model.stride.max())
colors = ((50, 50, 50), (0, 0, 255), (0, 255, 0)) # (gray, red, green)

# 차량,번호판 모델
CONFIDENCE = 0.5
THRESHOLD = 0.3
LABELS = ['차량', '번호판']

net = cv2.dnn.readNetFromDarknet('models/yolov4-ANPR.cfg', 'models/yolov4-ANPR.weights')

# 동영상 로드
cap = cv2.VideoCapture('data/full3.mp4')

fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
out = cv2.VideoWriter('data/output.mp4', fourcc, cap.get(cv2.CAP_PROP_FPS), (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))

while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        break

    H, W, _ = img.shape

    # preprocess
    img_input = letterbox(img, img_size, stride=stride)[0]
    img_input = img_input.transpose((2, 0, 1))[::-1]
    img_input = np.ascontiguousarray(img_input)
    img_input = torch.from_numpy(img_input).to(device)
    img_input = img_input.float()
    img_input /= 255.
    img_input = img_input.unsqueeze(0)

    # inference 횡단보도,신호등
    pred = model(img_input, augment=False, visualize=False)[0]

    # postprocess
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)[0]

    pred = pred.cpu().numpy()

    pred[:, :4] = scale_coords(img_input.shape[2:], pred[:, :4], img.shape).round()

    # inference 차량,번호판
    blob = cv2.dnn.blobFromImage(img, scalefactor=1/255., size=(416, 416), swapRB=True)
    net.setInput(blob)
    output = net.forward()

    boxes, confidences, class_ids = [], [], []

    for det in output:
        box = det[:4]
        scores = det[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]

        if confidence > CONFIDENCE:
            cx, cy, w, h = box * np.array([W, H, W, H])
            x = cx - (w / 2)
            y = cy - (h / 2)

            boxes.append([int(x), int(y), int(w), int(h)])
            confidences.append(float(confidence))
            class_ids.append(class_id)

    idxs = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE, THRESHOLD)

    # Visualize
    annotator = Annotator(img.copy(), line_width=3, example=str(class_names), font='data/malgun.ttf')

    cw_x1, cw_x2 = None, None # 횡단보도 좌측(cw_x1), 우측(cw_x2) 좌표

    for p in pred:
        class_name = class_names[int(p[5])]
        x1, y1, x2, y2 = p[:4]

        annotator.box_label([x1, y1, x2, y2], '%s %d' % (class_name, float(p[4]) * 100), color=colors[int(p[5])])

        if class_name == '횡단보도':
            cw_x1, cw_x2 = x1, x2

    if len(idxs) > 0:
        for i in idxs.flatten():
            class_name = LABELS[class_ids[i]]
            x1, y1, w, h = boxes[i]

        alert_text = ''
        color = (255, 0, 0) # blue

        if class_name == '차량':
            if x1 < cw_x2: # 차량의 좌측(x1) 좌표가 횡단보도의 우측(cw_x2)을 침범하였을 경우
                alert_text = '[!위반]'
                color = (0, 0, 255) # red

        annotator.box_label([x1, y1, x1+w, y1+h], '%s %d' % (alert_text+class_name, confidences[i] * 100), color=color)

    result_img = annotator.result()

    cv2.imshow('result', result_img)
    out.write(result_img)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
out.release()
