#!/usr/bin/python3
import lib.advanced_lane_finding.finding_lines as fl
import lib.advanced_lane_finding.threshold as thr
import numpy as np
import cv2

def test(image):
    # extract edge and specific color
    th_sobelx, th_sobely, th_mag, th_dir = (35, 100), (30, 255), (30, 255), (0.7, 1.3)
    th_h, th_l, th_s = (10, 100), (0, 60), (85, 255)  # h: hue, l: lightnes, s: saturation
    combined_gradient = thr.gradient_combine(image, th_sobelx, th_sobely, th_mag, th_dir)
    combined_hls = thr.hls_combine(image, th_h, th_l, th_s)
    combined_result = thr.comb_result(combined_gradient, combined_hls)
    cv2.imshow('edge',combined_result)

    # TODO: magic number festival
    c_rows, c_cols = combined_result.shape[:2]
    s_LTop2, s_RTop2 = [c_cols / 2 - 24, 5], [c_cols / 2 + 24, 5]   # upper little-left , upper little-right
    s_LBot2, s_RBot2 = [110, c_rows], [c_cols - 110, c_rows]        # lower left, lower right
    src = np.float32([s_LBot2, s_LTop2, s_RTop2, s_RBot2])
    dst = np.float32([(170, 720), (170, 0), (550, 0), (550, 720)])

    # Warp image to make bird eye view
    warp_img, M, Minv = fl.warp_image(combined_result, src, dst, (720, 720))
    cv2.imshow('warp',warp_img)    

    # 
    left_line = fl.Line()
    right_line = fl.Line()
    searching_img = fl.find_LR_lines(warp_img, left_line, right_line)
    w_comb_result, w_color_result = fl.draw_lane(searching_img, left_line, right_line)

    # Drawing the lines
    rows, cols = image.shape[:2]
    color_result = cv2.warpPerspective(w_color_result, Minv, (c_cols, c_rows))
    comb_result = np.zeros_like(image)
    comb_result[thr.MIDDLE:rows - 12, 0:cols] = color_result

    # Combine the result with the original image
    result = cv2.addWeighted(image, 1, comb_result, 0.3, 0)
    return result