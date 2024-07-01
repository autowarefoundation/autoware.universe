#!/usr/bin/env bash
#weights="enhance/pretrained/yolov5s.pt"
weights="enhance/pretrained/face_person.pt"
python export.py \
  --weights $weights \
  --img 640 \
  --batch 1  # export at 640x640 with batch size 1
