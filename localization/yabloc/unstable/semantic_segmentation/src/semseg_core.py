#!/usr/bin/env python3
import copy
import numpy as np
import cv2
import os


class SemSeg:
    def __init__(self, model_path):
        self.net_ = cv2.dnn.readNet(model_path)
        self.net_.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net_.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def makeBlob(self, image: np.ndarray) -> np.ndarray:
        scale = 1.0
        size = (896, 512)
        mean = (0, 0, 0)
        swap = True
        crop = False
        depth = cv2.CV_32F
        return cv2.dnn.blobFromImage(image, scale, size, mean, swap, crop, depth)

    def inference(self, image: np.ndarray) -> np.ndarray:
        blob = self.makeBlob(image)
        self.net_.setInput(blob)
        output_layers = self.net_.getUnconnectedOutLayersNames()
        mask = self.net_.forward(output_layers)[0]
        mask = np.squeeze(mask).transpose(1, 2, 0)

        mask = cv2.resize(
            mask,
            dsize=(image.shape[1], image.shape[0]),
            interpolation=cv2.INTER_LINEAR
        )

        return mask

    def drawOverlayMask(self, image, segmentation, score_threshold=0.5) -> np.ndarray:
        overlay_image = copy.deepcopy(image)

        masks = cv2.split(segmentation)[1:]
        bin_masks = []
        for mask in masks:
            bin_mask = np.where(mask > score_threshold, 0, 1).astype('uint8')
            bin_masks.append(255-255*bin_mask)
        mask_image = cv2.merge(bin_masks)
        overlay_image = cv2.addWeighted(overlay_image, 0.5, mask_image, 0.5, 1.0)
        return overlay_image


def main():
    dirname = os.path.dirname(__file__)
    path = dirname+'/../data/model_32.pb'


if __name__ == '__main__':
    main()
