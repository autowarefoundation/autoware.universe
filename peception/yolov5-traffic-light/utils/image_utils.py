# -*-coding: utf-8 -*-
"""
    @Project: IntelligentManufacture
    @File   : image_processing.py
    @Author : panjq
    @E-mail : pan_jinquan@163.com
    @Date   : 2019-02-14 15:34:50
"""

import os
import copy
import re
import cv2
import numpy as np
import matplotlib.pyplot as plt
import requests
import matplotlib
import numbers
import base64
import PIL.Image as Image
from PIL import ImageDraw, ImageFont
from math import cos, sin


def get_color_map(nums=25):
    colors = [
        "#FF0000", "#0000FF", "#00FF00", "#FFFF00", "#FF00FF",
        "#FF7F50", "#FF8000", "#FF9912", "#DAA569", "#FF6100",
        "#872657", "#3D59AB", "#03A89E", "#33A1C9", "#00C78C",
        "#B0171F", "#385E0F", "#00C957", "#6B8E23", "#2E8B57",
        "#A020F0", "#8A2BE2", "#A066D3", "#DA70D6", "#DDA0DD"]
    colors = colors * int(np.ceil(nums / len(colors)))
    return colors


def get_color(id):
    color = convert_color_map(COLOR_MAP[id])
    return color


def set_class_set(class_set=set()):
    global CLASS_SET
    CLASS_SET = class_set


COLOR_MAP = get_color_map(200)
CLASS_SET = set()

cmap = plt.get_cmap('rainbow')


def get_colors(nums):
    colors = [cmap(i) for i in np.linspace(0, 1, nums + 2)]
    colors = [(c[2], c[1], c[0]) for c in colors]
    return colors


def convert_color_map(color, colorType="BGR"):
    '''
    :param color:
    :param colorType:
    :return:
    '''
    assert (len(color) == 7 and color[0] == "#"), "input color error:color={}".format(color)
    R = color[1:3]
    G = color[3:5]
    B = color[5:7]

    R = int(R, 16)
    G = int(G, 16)
    B = int(B, 16)
    if colorType == "BGR":
        return (B, G, R)
    elif colorType == "RGB":
        return (R, G, B)
    else:
        assert "colorType error "


def points_protection(points, height, width):
    """
    Avoid array overbounds
    :param points:
    :param height:
    :param width:
    :return:
    """
    if not isinstance(points, np.ndarray):
        points = np.asarray(points)
    # points[points[:, 0] > width, 0] = width - 1  # x
    # points[points[:, 1] > height, 1] = height - 1  # y
    # points[points[:, 0] < 0, 0] = 0  # x
    # points[points[:, 1] < 0, 1] = 0  # y
    if len(points) > 0:
        points[:, 0] = np.clip(points[:, 0], 0, width - 1)
        points[:, 1] = np.clip(points[:, 1], 0, height - 1)
    return points


def bboxes_protection(boxes, width, height):
    """
    :param boxes:
    :param width:
    :param height:
    :return:
    """
    if not isinstance(boxes, np.ndarray):
        boxes = np.asarray(boxes)
    if len(boxes) > 0:
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, width - 1)
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, height - 1)
    return boxes


def tensor2image(batch_tensor, index=0):
    """
    convert tensor to image
    :param batch_tensor:
    :param index:
    :return:
    """
    image_tensor = batch_tensor[index, :]
    image = np.array(image_tensor, dtype=np.float32)
    image = np.squeeze(image)
    image = image.transpose(1, 2, 0)  # 通道由[c,h,w]->[h,w,c]
    return image


def get_image_tensor(image_path, image_size, transpose=False):
    image = read_image(image_path)
    # transform = default_transform(image_size)
    # torch_image = transform(image).detach().numpy()
    image = resize_image(image, int(128 * image_size[0] / 112), int(128 * image_size[1] / 112))
    image = center_crop(image, crop_size=image_size)
    image_tensor = image_normalization(image, mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
    if transpose:
        image_tensor = image_tensor.transpose(2, 0, 1)  # NHWC->NCHW
    image_tensor = image_tensor[np.newaxis, :]
    # std = np.std(torch_image-image_tensor)
    return image_tensor


def image_clip(image):
    """
    :param image:
    :return:
    """
    image = np.clip(image, 0, 1)
    return image


def transpose(data):
    data = data.transpose(2, 0, 1)  # HWC->CHW
    return data


def untranspose(data):
    if len(data.shape) == 3:
        data = data.transpose(1, 2, 0).copy()  # 通道由[c,h,w]->[h,w,c]
    else:
        data = data.transpose(1, 0).copy()
    return data


def pointPolygonTest(point, contour, measureDist=False):
    """
    此功能可查找图像中的点与轮廓之间的最短距离.
    当点在轮廓外时返回负值，当点在内部时返回正值，如果点在轮廓上则返回零
    :param point: (x,y)
    :param contour: np.asarray,(num_point,2)
    :param measureDist: 如果为True，则查找签名距离(signed distance)
                        如果为False，则查找该点是在内部还是外部或在轮廓上（它分别返回+1，-1,0）
    果您不想找到距离，请设置measureDist=False，因为这是一个耗时的过程. 因此，将其设为False可提供2-3倍的加速.
    :return:
    """
    contour = np.asarray(contour, dtype=np.int32)
    point = (int(point[0]), int(point[1]))
    dist = cv2.pointPolygonTest(contour, point, measureDist)
    return dist


def show_batch_image(title, batch_imgs, index=0):
    '''
    批量显示图片
    :param title:
    :param batch_imgs:
    :param index:
    :return:
    '''
    image = batch_imgs[index, :]
    # image = image.numpy()  #
    image = np.array(image, dtype=np.float32)
    image = np.squeeze(image)
    image = untranspose(image)
    if title:
        cv_show_image(title, image)


def show_image(title, rgb_image):
    '''
    调用matplotlib显示RGB图片
    :param title: 图像标题
    :param rgb_image: 图像的数据
    :return:
    '''
    # plt.figure("show_image")
    # print(image.dtype)
    channel = len(rgb_image.shape)
    if channel == 3:
        plt.imshow(rgb_image)
    else:
        plt.imshow(rgb_image, cmap='gray')
    plt.axis('on')  # 关掉坐标轴为 off
    plt.title(title)  # 图像题目
    plt.show()


def cv_show_image(title, image, type='rgb', waitKey=0):
    '''
    调用OpenCV显示RGB图片
    :param title: 图像标题
    :param image: 输入RGB图像
    :param type:'rgb' or 'bgr'
    :return:
    '''
    img = copy.copy(image)
    channels = img.shape[-1]
    if channels == 3 and type == 'rgb':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # 将BGR转为RGB
    if title:
        # cv2.namedWindow(title, flags=cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(title, flags=cv2.WINDOW_NORMAL)
        cv2.imshow(title, img)
        cv2.waitKey(waitKey)
    return img


def show_images_list(name, images_list, waitKey=0):
    out = image_hstack(images_list)
    cv2.imshow(name, out)
    cv2.waitKey(waitKey)


def image_hstack(images):
    """图像左右拼接"""
    img_new = images[0]
    shape = img_new.shape
    for i in range(1, len(images)):
        img = images[i]
        if not shape == img.shape:
            img = cv2.resize(img, dsize=(shape[1], shape[0]))
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img_new = np.hstack((img_new, img))
    return img_new


def image_vstack(images):
    """图像上下拼接"""
    img_new = images[0]
    shape = img_new.shape
    for i in range(1, len(images)):
        img = images[i]
        if not shape == img.shape:
            img = cv2.resize(img, dsize=(shape[1], shape[0]))
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img_new = np.vstack((img_new, img))
    return img_new


def image_fliplr(image):
    '''
    左右翻转
    :param image:
    :return:
    '''
    image = np.fliplr(image)
    return image


def get_prewhiten_image(x):
    '''
    图片白化处理
    :param x:
    :return:
    '''
    mean = np.mean(x)
    std = np.std(x)
    std_adj = np.maximum(std, 1.0 / np.sqrt(x.size))
    y = np.multiply(np.subtract(x, mean), 1 / std_adj)
    return y


def image_normalization(image, mean=None, std=None):
    '''
    正则化，归一化
    image[channel] = (image[channel] - mean[channel]) / std[channel]
    :param image: numpy image
    :param mean: [0.5,0.5,0.5]
    :param std:  [0.5,0.5,0.5]
    :return:
    '''
    # 不能写成:image=image/255
    if isinstance(mean, list):
        mean = np.asarray(mean, dtype=np.float32)
    if isinstance(std, list):
        std = np.asarray(std, dtype=np.float32)
    image = np.array(image, dtype=np.float32)
    image = image / 255.0
    if mean is not None:
        image = np.subtract(image, mean)
    if std is not None:
        image = np.multiply(image, 1 / std)
    return image


def image_unnormalization(image, mean=None, std=None):
    '''
    正则化，归一化
    image[channel] = (image[channel] - mean[channel]) / std[channel]
    :param image: numpy image
    :param mean: [0.5,0.5,0.5]
    :param std:  [0.5,0.5,0.5]
    :return:
    '''
    # 不能写成:image=image/255
    if isinstance(mean, list):
        mean = np.asarray(mean, dtype=np.float32)
    if isinstance(std, list):
        std = np.asarray(std, dtype=np.float32)
    image = np.multiply(image, std)
    image = (image + mean) * 255
    image = np.array(image, dtype=np.uint8)
    return image


def data_normalization(data, omin, omax, imin=None, imax=None):
    '''
    NORMALIZATION 将数据x归一化到任意区间[ymin,omax]范围的方法
    :param data:  输入参数x：需要被归一化的数据,numpy
    :param omin: 输入参数omin：归一化的区间[omin,omax]下限
    :param omax: 输入参数ymax：归一化的区间[omin,omax]上限
    :param imin: 输入参数imin的最小值
    :param imax: 输入参数ymax的最大值
    :return: 输出参数y：归一化到区间[omin,omax]的数据
    '''
    imax = imax if imax is not None else np.max(data)  # %计算最大值
    imin = imin if imin is not None else np.min(data)  # %计算最小值
    y = (omax - omin) * (data - imin) / (imax - imin) + omin
    return y


def cv_image_normalization(image, min_val=0.0, max_val=1.0):
    '''

    :param image:
    :param min_val:
    :param max_val:
    :param norm_type:
    :param dtype:
    :param mask:
    :return:
    '''
    dtype = cv2.CV_32F
    norm_type = cv2.NORM_MINMAX
    out = np.zeros(shape=image.shape, dtype=np.float32)
    cv2.normalize(image, out, alpha=min_val, beta=max_val, norm_type=norm_type, dtype=dtype)
    return out


def get_prewhiten_images(images_list, normalization=False):
    '''
    批量白化图片处理
    :param images_list:
    :param normalization:
    :return:
    '''
    out_images = []
    for image in images_list:
        if normalization:
            image = image_normalization(image)
        image = get_prewhiten_image(image)
        out_images.append(image)
    return out_images


def read_image(filename, resize_height=None, resize_width=None, normalization=False, colorSpace='RGB'):
    '''
    读取图片数据,默认返回的是uint8,[0,255]
    :param filename:
    :param resize_height:
    :param resize_width:
    :param normalization:是否归一化到[0.,1.0]
    :param colorSpace 输出格式：RGB or BGR
    :return: 返回的图片数据
    '''

    bgr_image = cv2.imread(filename)
    # bgr_image = cv2.imread(filename,cv2.IMREAD_IGNORE_ORIENTATION|cv2.IMREAD_UNCHANGED)
    if bgr_image is None:
        print("Warning: no image:{}".format(filename))
        return None
    if len(bgr_image.shape) == 2:  # 若是灰度图则转为三通道
        print("Warning:gray image", filename)
        bgr_image = cv2.cvtColor(bgr_image, cv2.COLOR_GRAY2BGR)

    if colorSpace == 'RGB':
        image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)  # 将BGR转为RGB
    elif colorSpace == "BGR":
        image = bgr_image
    else:
        exit(0)
    # show_image(filename,image)
    # image=Image.open(filename)
    image = resize_image(image, resize_height, resize_width)
    image = np.asanyarray(image)
    if normalization:
        image = image_normalization(image)
    # show_image("src resize image",image)
    return image


def read_image_pil(filename, resize_height=None, resize_width=None, normalization=False):
    '''
    读取图片数据,默认返回的是uint8,[0,255]
    :param filename:
    :param resize_height:
    :param resize_width:
    :param normalization:是否归一化到[0.,1.0]
    :param colorSpace 输出格式：RGB or BGR
    :return: 返回的图片数据
    '''

    rgb_image = Image.open(filename)
    rgb_image = np.asarray(rgb_image)
    if rgb_image is None:
        print("Warning: no image:{}".format(filename))
        return None
    if len(rgb_image.shape) == 2:  # 若是灰度图则转为三通道
        print("Warning:gray image", filename)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_GRAY2BGR)

    # show_image(filename,image)
    # image=Image.open(filename)
    image = resize_image(rgb_image, resize_height, resize_width)
    if normalization:
        image = image_normalization(image)
    # show_image("src resize image",image)
    return image


def read_image_gbk(filename, resize_height=None, resize_width=None, normalization=False, colorSpace='RGB'):
    '''
    解决imread不能读取中文路径的问题,读取图片数据,默认返回的是uint8,[0,255]
    :param filename:
    :param resize_height:
    :param resize_width:
    :param normalization:是否归一化到[0.,1.0]
    :param colorSpace 输出格式：RGB or BGR
    :return: 返回的RGB图片数据
    '''
    try:
        with open(filename, 'rb') as f:
            data = f.read()
            data = np.asarray(bytearray(data), dtype="uint8")
            bgr_image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    except Exception as e:
        bgr_image = None
    # 或者：
    # bgr_image=cv2.imdecode(np.fromfile(filename,dtype=np.uint8),cv2.IMREAD_COLOR)
    if bgr_image is None:
        print("Warning: no image:{}".format(filename))
        return None
    if len(bgr_image.shape) == 2:  # 若是灰度图则转为三通道
        print("Warning:gray image", filename)
        bgr_image = cv2.cvtColor(bgr_image, cv2.COLOR_GRAY2BGR)
    if colorSpace == 'RGB':
        image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)  # 将BGR转为RGB
    elif colorSpace == "BGR":
        image = bgr_image
    else:
        exit(0)
    # show_image(filename,image)
    # image=Image.open(filename)
    image = resize_image(image, resize_height, resize_width)
    image = np.asanyarray(image)
    if normalization:
        image = image_normalization(image)
    # show_image("src resize image",image)
    return image


def requests_url(url):
    '''
    读取网络数据流
    :param url:
    :return:
    '''
    stream = None
    try:
        res = requests.get(url, timeout=15)
        if res.status_code == 200:
            stream = res.content
    except Exception as e:
        print(e)
    return stream


def read_images_url(url, resize_height=None, resize_width=None, normalization=False, colorSpace='RGB'):
    '''
    根据url或者图片路径，读取图片
    :param url:
    :param resize_height:
    :param resize_width:
    :param normalization:
    :param colorSpace:
    :return:
    '''
    if re.match(r'^https?:/{2}\w.+$', url):
        stream = requests_url(url)
        if stream is None:
            bgr_image = None
        else:
            content = np.asarray(bytearray(stream), dtype="uint8")
            bgr_image = cv2.imdecode(content, cv2.IMREAD_COLOR)
            # pil_image = PIL.Image.open(BytesIO(stream))
            # rgb_image=np.asarray(pil_image)
            # bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    else:
        bgr_image = cv2.imread(url)

    if bgr_image is None:
        print("Warning: no image:{}".format(url))
        return None
    if len(bgr_image.shape) == 2:  # 若是灰度图则转为三通道
        print("Warning:gray image", url)
        bgr_image = cv2.cvtColor(bgr_image, cv2.COLOR_GRAY2BGR)
    if colorSpace == 'RGB':
        image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)  # 将BGR转为RGB
    elif colorSpace == "BGR":
        image = bgr_image
    else:
        pass
    image = resize_image(image, resize_height, resize_width)
    image = np.asanyarray(image)
    if normalization:
        image = image_normalization(image)
    # show_image("src resize image",image)
    return image


def read_image_batch(image_list):
    '''
    批量读取图片
    :param image_list:
    :return:
    '''
    image_batch = []
    out_image_list = []
    for image_path in image_list:
        image = read_images_url(image_path)
        if image is None:
            print("no image:{}".format(image_path))
            continue
        image_batch.append(image)
        out_image_list.append(image_path)
    return image_batch, out_image_list


def fast_read_image_roi(filename, orig_rect, ImreadModes=cv2.IMREAD_COLOR, normalization=False, colorSpace='RGB'):
    '''
    快速读取图片的方法
    :param filename: 图片路径
    :param orig_rect:原始图片的感兴趣区域rect
    :param ImreadModes: IMREAD_UNCHANGED
                        IMREAD_GRAYSCALE
                        IMREAD_COLOR
                        IMREAD_ANYDEPTH
                        IMREAD_ANYCOLOR
                        IMREAD_LOAD_GDAL
                        IMREAD_REDUCED_GRAYSCALE_2
                        IMREAD_REDUCED_COLOR_2
                        IMREAD_REDUCED_GRAYSCALE_4
                        IMREAD_REDUCED_COLOR_4
                        IMREAD_REDUCED_GRAYSCALE_8
                        IMREAD_REDUCED_COLOR_8
                        IMREAD_IGNORE_ORIENTATION
    :param normalization: 是否归一化
    :param colorSpace 输出格式：RGB or BGR
    :return: 返回感兴趣区域ROI
    '''
    # 当采用IMREAD_REDUCED模式时，对应rect也需要缩放
    scale = 1
    if ImreadModes == cv2.IMREAD_REDUCED_COLOR_2 or ImreadModes == cv2.IMREAD_REDUCED_COLOR_2:
        scale = 1 / 2
    elif ImreadModes == cv2.IMREAD_REDUCED_GRAYSCALE_4 or ImreadModes == cv2.IMREAD_REDUCED_COLOR_4:
        scale = 1 / 4
    elif ImreadModes == cv2.IMREAD_REDUCED_GRAYSCALE_8 or ImreadModes == cv2.IMREAD_REDUCED_COLOR_8:
        scale = 1 / 8
    rect = np.array(orig_rect) * scale
    rect = rect.astype(int).tolist()
    bgr_image = cv2.imread(filename, flags=ImreadModes)

    if bgr_image is None:
        print("Warning: no image:{}".format(filename))
        return None
    if len(bgr_image.shape) == 2:  # 若是灰度图则转为三通道
        print("Warning:gray image", filename)
        bgr_image = cv2.cvtColor(bgr_image, cv2.COLOR_GRAY2BGR)
    if colorSpace == 'RGB':
        image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)  # 将BGR转为RGB
    elif colorSpace == "BGR":
        image = bgr_image
    image = np.asanyarray(image)
    if normalization:
        image = image_normalization(image)
    roi_image = get_rect_image(image, rect)
    return roi_image


def resize_image(image, resize_height=None, resize_width=None):
    '''
    tf.image.resize_images(images,size),images=[batch, height, width, channels],size=(new_height, new_width)
    cv2.resize(image, dsize=(resize_width, resize_height)),与image.shape相反
    images[50,10]与image.shape的原理相同，它表示的是image=(y=50,x=10)
    :param image:
    :param resize_height:
    :param resize_width:
    :return:
    '''
    image_shape = np.shape(image)
    height = image_shape[0]
    width = image_shape[1]
    if (resize_height is None) and (resize_width is None):  # 错误写法：resize_height and resize_width is None
        return image
    if resize_height is None:
        resize_height = int(height * resize_width / width)
    elif resize_width is None:
        resize_width = int(width * resize_height / height)
    image = cv2.resize(image, dsize=(resize_width, resize_height))
    return image


def image_boxes_resize_padding(image, input_size, boxes=None, color=(0, 0, 0)):
    """
    等比例图像resize,保持原始图像内容比，避免失真,短边会0填充
    input_size = [300, 300]
    image_path = "test.jpg"
    src_boxes = [[8.20251, 1, 242.2412, 699.2236],
                 [201.14865, 204.18265, 468.605, 696.36163]]
    src_boxes = np.asarray(src_boxes)
    image = read_image(image_path)
    image1, boxes1 = image_boxes_resize_padding(image, input_size, src_boxes)
    image1 = show_image_boxes("Det", image1, boxes1, color=(255, 0, 0), waitKey=3)
    boxes = image_boxes_resize_padding_inverse(image.shape, input_size, boxes1)
    show_image_boxes("image", image, boxes)
    :param size:
    """
    height, width = image.shape[:2]
    scale = min([input_size[0] / width, input_size[1] / height])
    new_size = [int(width * scale), int(height * scale)]
    pad_w = input_size[0] - new_size[0]
    pad_h = input_size[1] - new_size[1]
    top, bottom = pad_h // 2, pad_h - (pad_h // 2)
    left, right = pad_w // 2, pad_w - (pad_w // 2)
    out = cv2.resize(image, (new_size[0], new_size[1]))
    out = cv2.copyMakeBorder(out, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    if not boxes is None and len(boxes) > 0:
        boxes[:] = boxes[:] * scale
        boxes[:] = boxes[:] + [left, top, left, top]
    return out


def image_boxes_resize_padding_inverse(image_size, input_size, boxes=None):
    """
    resize_image_padding的逆过程
    """
    width, height = image_size
    scale = min([input_size[0] / width, input_size[1] / height])
    new_size = [int(width * scale), int(height * scale)]
    pad_w = input_size[0] - new_size[0]
    pad_h = input_size[1] - new_size[1]
    top, bottom = pad_h // 2, pad_h - (pad_h // 2)
    left, right = pad_w // 2, pad_w - (pad_w // 2)
    if not boxes is None and len(boxes) > 0:
        boxes[:] = boxes[:] - [left, top, left, top]
        boxes[:] = boxes[:] / scale
    return boxes


def resize_image_bboxes(image, resize_height=None, resize_width=None, bboxes=None):
    """
    :param image:
    :param resize_height:
    :param resize_width:
    :param bboxes:
    :return:
    """
    height, width, _ = image.shape
    if (resize_height is None) and (resize_width is None):  # 错误写法：resize_height and resize_width is None
        return image, bboxes
    if resize_height is None:
        scale = [resize_width / width, resize_width / width]
        resize_height = int(height * resize_width / width)
    elif resize_width is None:
        scale = [resize_height / height, resize_height / height]
        resize_width = int(width * resize_height / height)
    else:
        scale = [resize_width / width, resize_height / height]
    bboxes = scale * 2 * bboxes
    image = cv2.resize(image, dsize=(resize_width, resize_height))
    return image, bboxes


def scale_image(image, scale):
    '''
    :param image:
    :param scale: (scale_w,scale_h)
    :return:
    '''
    image = cv2.resize(image, dsize=None, fx=scale[0], fy=scale[1])
    return image


def get_rect_image(image, rect):
    '''
    :param image:
    :param rect: [x,y,w,h]
    :return:
    '''
    shape = image.shape  # h,w
    height = shape[0]
    width = shape[1]
    image_rect = (0, 0, width, height)
    rect = get_rect_intersection(rect, image_rect)
    rect = [int(i) for i in rect]
    x, y, w, h = rect
    cut_img = image[y:(y + h), x:(x + w)]
    return cut_img


def get_rects_image(image, rects_list, resize_height=None, resize_width=None):
    '''
    获得裁剪区域
    :param image:
    :param rects_list:
    :param resize_height:
    :param resize_width:
    :return:
    '''
    rect_images = []
    for rect in rects_list:
        roi = get_rect_image(image, rect)
        roi = resize_image(roi, resize_height, resize_width)
        rect_images.append(roi)
    return rect_images


def get_bboxes_image(image, bboxes_list, resize_height=None, resize_width=None):
    '''
    获得裁剪区域
    :param image:
    :param bboxes_list:
    :param resize_height:
    :param resize_width:
    :return:
    '''
    rects_list = bboxes2rects(bboxes_list)
    rect_images = get_rects_image(image, rects_list, resize_height, resize_width)
    return rect_images


def bboxes2rects(bboxes_list):
    '''
    将bboxes=[x1,y1,x2,y2] 转为rect=[x1,y1,w,h]
    :param bboxes_list:
    :return:
    '''
    rects_list = []
    for bbox in bboxes_list:
        x1, y1, x2, y2 = bbox
        rect = [x1, y1, (x2 - x1), (y2 - y1)]
        rects_list.append(rect)
    return rects_list


def rects2bboxes(rects_list):
    '''
    将rect=[x1,y1,w,h]转为bboxes=[x1,y1,x2,y2]
    :param rects_list:
    :return:
    '''
    bboxes_list = []
    for rect in rects_list:
        x1, y1, w, h = rect
        x2 = x1 + w
        y2 = y1 + h
        b = (x1, y1, x2, y2)
        bboxes_list.append(b)
    return bboxes_list


def bboxes2center(bboxes_list):
    '''
    center = (boxes[:, [0, 1]] + boxes[:, [2, 3]]) / 2
    将bboxes=[x1,y1,x2,y2] 转为center_list=[cx,cy,w,h]
    :param bboxes_list:
    :return:
    '''
    center_list = []
    for bbox in bboxes_list:
        x1, y1, x2, y2 = bbox
        center = [(x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1), (y2 - y1)]
        center_list.append(center)
    return center_list


def center2bboxes(center_list):
    '''
    将center_list=[cx,cy,w,h] 转为bboxes=[x1,y1,x2,y2]
    :param bboxes_list:
    :return:
    '''
    bboxes_list = []
    for c in center_list:
        cx, cy, w, h = c
        bboxes = [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]
        bboxes_list.append(bboxes)
    return bboxes_list


def center2rects(center_list):
    '''
    将center_list=[cx,cy,w,h] 转为rect=[x,y,w,h]
    :param bboxes_list:
    :return:
    '''
    rect_list = []
    for c in center_list:
        cx, cy, w, h = c
        rect = [cx - w / 2, cy - h / 2, w, h]
        rect_list.append(rect)
    return rect_list


def scale_rect(orig_rect, orig_shape, dest_shape):
    '''
    对图像进行缩放时，对应的rectangle也要进行缩放
    :param orig_rect: 原始图像的rect=[x,y,w,h]
    :param orig_shape: 原始图像的维度shape=[h,w]
    :param dest_shape: 缩放后图像的维度shape=[h,w]
    :return: 经过缩放后的rectangle
    '''
    new_x = int(orig_rect[0] * dest_shape[1] / orig_shape[1])
    new_y = int(orig_rect[1] * dest_shape[0] / orig_shape[0])
    new_w = int(orig_rect[2] * dest_shape[1] / orig_shape[1])
    new_h = int(orig_rect[3] * dest_shape[0] / orig_shape[0])
    dest_rect = [new_x, new_y, new_w, new_h]
    return dest_rect


def get_rect_intersection(rec1, rec2):
    '''
    计算两个rect的交集坐标
    :param rec1:
    :param rec2:
    :return:
    '''
    xmin1, ymin1, xmax1, ymax1 = rects2bboxes([rec1])[0]
    xmin2, ymin2, xmax2, ymax2 = rects2bboxes([rec2])[0]
    x1 = max(xmin1, xmin2)
    y1 = max(ymin1, ymin2)
    x2 = min(xmax1, xmax2)
    y2 = min(ymax1, ymax2)
    w = max(0, x2 - x1)
    h = max(0, y2 - y1)
    return (x1, y1, w, h)


def get_bbox_intersection(box1, box2):
    '''
    计算两个boxes的交集坐标
    :param rec1:
    :param rec2:
    :return:
    '''
    xmin1, ymin1, xmax1, ymax1 = box1
    xmin2, ymin2, xmax2, ymax2 = box2
    x1 = max(xmin1, xmin2)
    y1 = max(ymin1, ymin2)
    x2 = min(xmax1, xmax2)
    y2 = min(ymax1, ymax2)
    return (x1, y1, x2, y2)


def draw_image_rects(bgr_image, rect_list, color=(0, 0, 255), thickness=2):
    for rect in rect_list:
        x, y, w, h = rect
        point1 = (int(x), int(y))
        point2 = (int(x + w), int(y + h))
        cv2.rectangle(bgr_image, point1, point2, color, thickness=thickness)
    return bgr_image


def draw_image_boxes(bgr_image, boxes_list, color=(0, 0, 255), thickness=1):
    for box in boxes_list:
        x1, y1, x2, y2 = box[:4]
        point1 = (int(x1), int(y1))
        point2 = (int(x2), int(y2))
        cv2.rectangle(bgr_image, point1, point2, color, thickness=thickness)
    return bgr_image


def show_image_rects(win_name, image, rect_list, type="rgb", color=(0, 0, 255), waitKey=0):
    '''
    :param win_name:
    :param image:
    :param rect_list:[[ x, y, w, h],[ x, y, w, h]]
    :return:
    '''
    image = draw_image_rects(image.copy(), rect_list, color)
    cv_show_image(win_name, image, type, waitKey=waitKey)
    return image


def show_image_boxes(win_name, image, boxes_list, color=(0, 0, 255), waitKey=0):
    '''
    :param win_name:
    :param image:
    :param boxes_list:[[ x1, y1, x2, y2],[ x1, y1, x2, y2]]
    :return:
    '''
    image = draw_image_boxes(image, boxes_list, color)
    cv_show_image(win_name, image, waitKey=waitKey)
    return image


def draw_image_bboxes_text(rgb_img, boxes, boxes_name, color=None, drawType="custom", top=True):
    """
    :param boxes_name:
    :param bgr_image: bgr image
    :param color: BGR color:[B,G,R]
    :param boxes: [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    :return:
    """
    rgb_image = rgb_img.copy()
    # color_map=list(matplotlib.colors.cnames.values())
    # color_map=list(reversed(color_map))
    if isinstance(boxes_name, np.ndarray):
        boxes_name = boxes_name.reshape(-1).tolist()
    if not color:
        class_set = list(CLASS_SET) if CLASS_SET else list(set(boxes_name))
    set_color = color
    for name, box in zip(boxes_name, boxes):
        if not color:
            cls_id = class_set.index(name)
            set_color = get_color(cls_id)
        box = [int(b) for b in box]
        # cv2.rectangle(bgr_image, (crop_type[0], crop_type[1]), (crop_type[2], crop_type[3]), (0, 255, 0), 2, 8, 0)
        # cv2.putText(bgr_image, name, (crop_type[0], crop_type[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), thickness=2)
        # cv2.rectangle(bgr_image, (crop_type[0], crop_type[1]), (crop_type[2], crop_type[3]), color, 2, 8, 0)
        # cv2.putText(bgr_image, str(name), (crop_type[0], crop_type[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, thickness=2)
        custom_bbox_line(rgb_image, box, set_color, name, drawType, top)
    # cv2.imshow(title, bgr_image)
    # cv2.waitKey(0)
    return rgb_image


def show_image_bboxes_text(title, rgb_image, boxes, boxes_name, color=None, drawType="custom", waitKey=0, top=True):
    '''
    :param boxes_name:
    :param bgr_image: bgr image
    :param color: BGR color:[B,G,R]
    :param boxes: [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    :return:
    '''
    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    bgr_image = draw_image_bboxes_text(bgr_image, boxes, boxes_name, color, drawType, top)
    rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    cv_show_image(title, rgb_image, waitKey=waitKey)
    return rgb_image


def draw_image_rects_text(rgb_image, rects, rects_name, color=None, drawType="custom", top=True):
    boxes = rects2bboxes(rects)
    rgb_image = draw_image_bboxes_text(rgb_image, boxes, rects_name, color, drawType, top)
    return rgb_image


def show_image_rects_text(title, rgb_image, rects, rects_name, color=None, drawType="custom", waitKey=0, top=True):
    '''
    :param rects_name:
    :param bgr_image: bgr image
    :param rects: [[x1,y1,w,h],[x1,y1,w,h]]
    :return:
    '''
    boxes = rects2bboxes(rects)
    rgb_image = show_image_bboxes_text(title, rgb_image, boxes, rects_name, color, drawType, waitKey, top)
    return rgb_image


def draw_image_detection_rects(rgb_image, rects, probs, labels, class_name=None):
    bboxes = rects2bboxes(rects)
    rgb_image = draw_image_detection_bboxes(rgb_image, bboxes, probs, labels, class_name)
    return rgb_image


def show_image_detection_rects(title, rgb_image, rects, probs, lables, color=None, waitKey=0):
    '''
    :param title:
    :param rgb_image:
    :param rects: [[x1,y1,w,h],[x1,y1,w,h]]
    :param probs:
    :param lables:
    :return:
    '''
    bboxes = rects2bboxes(rects)
    rgb_image = show_image_detection_bboxes(title, rgb_image, bboxes, probs, lables, color, waitKey)
    return rgb_image


def flat_data(data):
    if isinstance(data, np.ndarray):
        data = data.reshape(-1).tolist()
    return data


def draw_image_detection_bboxes(rgb_image, bboxes, probs, labels, class_name=None):
    '''
    :param title:
    :param rgb_image:
    :param bboxes:  [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    :param probs:
    :param labels:
    :return:
    '''
    labels = np.asarray(labels, dtype=np.int32).reshape(-1)
    probs = np.asarray(probs).reshape(-1)
    for label, box, prob in zip(labels, bboxes, probs):
        set_color = get_color(label)
        box = [int(b) for b in box]
        if class_name:
            label = class_name[label]
        boxes_name = "{}:{:3.3f}".format(label, prob)
        # cv2.rectangle(bgr_image, (crop_type[0], crop_type[1]), (crop_type[2], crop_type[3]), (0, 255, 0), 2, 8, 0)
        # cv2.putText(bgr_image, name, (crop_type[0], crop_type[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), thickness=2)
        # cv2.rectangle(bgr_image, (crop_type[0], crop_type[1]), (crop_type[2], crop_type[3]), color, 2, 8, 0)
        # cv2.putText(bgr_image, str(name), (crop_type[0], crop_type[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, thickness=2)
        custom_bbox_line(rgb_image, box, set_color, boxes_name, drawType="custom")
    # cv2.imshow(title, bgr_image)
    # cv2.waitKey(0)
    # rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    return rgb_image


def show_image_detection_bboxes(title, rgb_image, bboxes, probs, labels, class_name=None, waitKey=0):
    rgb_image = draw_image_detection_bboxes(rgb_image, bboxes, probs, labels, class_name)
    cv_show_image(title, rgb_image, waitKey=waitKey)
    return rgb_image


def draw_dt_gt_dets(image, dt_boxes, dt_label, gt_boxes, gt_label, vis_diff=False):
    """
    显示ground true和Detection bbox
    :param image:
    :param dt_boxes:
    :param dt_label:
    :param gt_boxes:
    :param gt_label:
    :param vis_diff: 是否显示差异，
                vis_diff=True :使用不同颜色标记gt_label和dt_label的差异
                               要求len(gt_label) == len(dt_label)
                vis_diff=False:使用不同颜色显示dt_boxes和gt_boxes
                                dt_boxes和gt_boxes的长度任意
    :return:
    """

    if vis_diff:
        assert len(gt_label) == len(dt_label)
        assert len(gt_boxes) == len(dt_boxes)
        for i in range(len(gt_label)):
            if dt_label[i] == gt_label[i]:
                image = draw_image_bboxes_text(image, [gt_boxes[i]], [gt_label[i]], color=(0, 255, 0))
                image = draw_image_bboxes_text(image, [dt_boxes[i]], [dt_label[i]], color=(0, 255, 0))
            else:
                image = draw_image_bboxes_text(image, [gt_boxes[i]], [gt_label[i]], color=(0, 255, 0))
                image = draw_image_bboxes_text(image, [dt_boxes[i]], [dt_label[i]], color=(255, 0, 0))
    else:
        image = draw_image_bboxes_text(image, gt_boxes, gt_label, color=(0, 255, 0))
        image = draw_image_bboxes_text(image, dt_boxes, dt_label, color=(255, 0, 0))
    return image


def custom_bbox_line(img, bbox, color, name, drawType="custom", top=True):
    """
    :param img:
    :param bbox:
    :param color:
    :param name:
    :param drawType:
    :param top:
    :return:
    """
    fontScale = 1
    thickness = 2
    if not name:
        drawType = "simple"
    if drawType == "simple":
        cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, thickness, 8, 0)
        cv2.putText(img, str(name), (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness)
    elif drawType == "custom":
        cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        # draw score roi
        # fontScale = 0.4
        text_size, baseline = cv2.getTextSize(str(name), cv2.FONT_HERSHEY_SIMPLEX, fontScale, thickness)
        if top:
            text_loc = (bbox[0], bbox[1] - text_size[1])
        else:
            # text_loc = (bbox[0], bbox[3])
            # text_loc = (bbox[2], bbox[3] - text_size[1])
            text_loc = (bbox[2], bbox[1] + text_size[1])

        cv2.rectangle(img, (text_loc[0] - 2 // 2, text_loc[1] - 2 - baseline),
                      (text_loc[0] + text_size[0], text_loc[1] + text_size[1]), color, -1)
        # draw score value
        cv2.putText(img, str(name), (text_loc[0], text_loc[1] + baseline), cv2.FONT_HERSHEY_SIMPLEX, fontScale,
                    (255, 255, 255), thickness, 4)
    return img


def show_boxList(win_name, boxList, rgb_image, waitKey=0):
    '''
    [xmin,ymin,xmax,ymax]
    :param win_name:
    :param boxList:
    :param rgb_image:
    :return:
    '''
    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    for item in boxList:
        name = item["label"]
        xmin = item["xtl"]
        xmax = item["xbr"]
        ymin = item["ytl"]
        ymax = item["ybr"]
        # crop_type=[xbr,ybr,xtl,ytl]
        box = [xmin, ymin, xmax, ymax]
        box = [int(float(b)) for b in box]
        cv2.rectangle(bgr_image, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2, 8, 0)
        cv2.putText(bgr_image, name, (box[0], box[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), thickness=2)
    # cv2.imshow(title, bgr_image)
    # cv2.waitKey(0)
    rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    if win_name:
        cv_show_image(win_name, rgb_image, waitKey=waitKey)
    return rgb_image


def draw_landmark(image, landmarks_list, color=(0, 0, 255), vis_id=False):
    image = copy.copy(image)
    point_size = 1
    thickness = 2  # 可以为 0 、4、8
    for landmarks in landmarks_list:
        for i, landmark in enumerate(landmarks):
            # 要画的点的坐标
            point = (int(landmark[0]), int(landmark[1]))
            cv2.circle(image, point, point_size, color, thickness)
            if vis_id:
                image = draw_points_text(image, [point], texts=[str(i)], color=color, thickness=thickness,
                                         drawType="simple")
    return image


def show_landmark_boxes(win_name, img, landmarks_list, boxes):
    '''
    显示landmark和boxex
    :param win_name:
    :param image:
    :param landmarks_list: [[x1, y1], [x2, y2]]
    :param boxes:     [[ x1, y1, x2, y2],[ x1, y1, x2, y2]]
    :return:
    '''
    image = draw_landmark(img, landmarks_list)
    image = show_image_boxes(win_name, image, boxes)
    return image


def show_landmark(win_name, img, landmarks_list, vis_id=False, waitKey=0):
    '''
    显示landmark和boxex
    :param win_name:
    :param image:
    :param landmarks_list: [[x1, y1], [x2, y2]]
    :return:
    '''
    image = draw_landmark(img, landmarks_list, vis_id=vis_id)
    cv_show_image(win_name, image, waitKey=waitKey)
    return image


def draw_points_text(img, points, texts=None, color=(255, 0, 0), thickness=1, drawType="simple"):
    '''

    :param img:
    :param points:
    :param texts:
    :param color:
    :param drawType: custom or simple
    :return:
    '''
    if texts is None:
        texts = [""] * len(points)
    for point, text in zip(points, texts):
        point = (int(point[0]), int(point[1]))
        cv2.circle(img, point, thickness * 2, color, -1)
        draw_text(img, point, text, bg_color=color, thickness=thickness, drawType=drawType)
    return img


def draw_text(img, point, text, bg_color=(255, 0, 0), thickness=5, drawType="custom"):
    '''
    :param img:
    :param point:
    :param text:
    :param drawType: custom or simple
    :return:
    '''
    fontScale = 0.5
    text_thickness = 1
    fontFace = cv2.FONT_HERSHEY_SIMPLEX
    # fontFace=cv2.FONT_HERSHEY_SIMPLEX
    if drawType == "custom":
        text_size, baseline = cv2.getTextSize(str(text), fontFace, fontScale, thickness)
        text_loc = (point[0], point[1] + text_size[1])
        cv2.rectangle(img, (text_loc[0] - 2 // 2, text_loc[1] - 2 - baseline),
                      (text_loc[0] + text_size[0], text_loc[1] + text_size[1]), color=bg_color, thickness=thickness)
        # draw score value
        cv2.putText(img, str(text), (text_loc[0], text_loc[1] + baseline), fontFace, fontScale,
                    (255, 255, 255), text_thickness, 2)
    elif drawType == "simple":
        cv2.putText(img, str(text), point, fontFace, fontScale, color=bg_color, thickness=thickness)
    return img


def draw_text_line(img, point, text_line: str, bg_color=(255, 0, 0), thickness=1, drawType="custom"):
    '''
    :param img:
    :param point:
    :param text:
    :param drawType: custom or custom
    :return:
    '''
    fontScale = 0.4
    fontFace = cv2.FONT_HERSHEY_SIMPLEX
    # fontFace=cv2.FONT_HERSHEY_SIMPLEX
    text_line = text_line.split("\n")
    # text_size, baseline = cv2.getTextSize(str(text_line), fontFace, fontScale, thickness)
    text_size, baseline = cv2.getTextSize(str(text_line), fontFace, fontScale, thickness)
    for i, text in enumerate(text_line):
        if text:
            draw_point = (point[0], point[1] + (text_size[1] + 2 + baseline) * i)
            img = draw_text(img, draw_point, text, bg_color, thickness=thickness, drawType=drawType)
    return img


def draw_text_pil(image, point, text, size=10, color_color=(255, 0, 0)):
    """
    支持显示中文
    :param image:
    :param point:
    :param text:
    :param size:
    :param color_color:
    :return:
    """
    pilimg = Image.fromarray(image)  # Image.fromarray()将数组类型转成图片格式，与np.array()相反
    draw = ImageDraw.Draw(pilimg)  # PIL图片上打印汉字
    # 参数1：字体文件路径，参数2：字体大小；Windows系统“simhei.ttf”默认存储在路径：
    font = ImageFont.truetype("./utils/simhei.ttf", size, encoding="utf-8")
    draw.text(point, text, color_color, font)
    image = cv2.cvtColor(np.array(pilimg), cv2.COLOR_RGB2BGR)  # 将图片转成cv2.imshow()可以显示的数组格式
    return image


def draw_key_point_in_image(image,
                            key_points,
                            pointline=[],
                            vis_id=False,
                            circle_color=(0, 255, 0),
                            line_color=(0, 0, 255),
                            thickness=2):
    '''
    :param key_points: list(ndarray(19,2)) or ndarray(n_person,19,2)
    :param image:
    :param pointline: `auto`->pointline = circle_line(len(points), iscircle=True)
    :return:
    '''
    img = copy.deepcopy(image)
    for person_id, points in enumerate(key_points):
        if points is None:
            continue
        if vis_id:
            text = None
        else:
            text = [""] * len(points)
        img = draw_image_points_lines(img, points, pointline,
                                      circle_color=circle_color,
                                      line_color=line_color,
                                      texts=text,
                                      thickness=thickness)
    return img


def draw_key_point_arrowed_in_image(image, key_points, pointline=[], color=None):
    '''
    :param key_points: list(ndarray(19,2)) or ndarray(n_person,19,2)
    :param image:
    :param pointline:[[start-->end]]
    :return:
    '''
    img = copy.deepcopy(image)
    person_nums = len(key_points)
    for person_id, points in enumerate(key_points):
        if points is None:
            continue
        if not color:
            set_color = get_color(person_id)
        else:
            set_color = color
        img = draw_image_points_arrowed_lines(img, points, pointline, set_color, check=True)
    return img


def draw_image_points_lines(image,
                            points,
                            pointline=[],
                            texts=None,
                            circle_color=(0, 255, 0),
                            line_color=(0, 0, 255),
                            thickness=2):
    '''
    在图像中画点和连接线
    :param img:
    :param points: 点列表
    :param pointline: `auto`->pointline = circle_line(len(points), iscircle=True)
    :param color:
    :param texts:
    :param drawType: simple or custom
    :param check:
    :return:
    '''
    points = np.asarray(points, dtype=np.int32)
    if texts is None:
        texts = list(range(len(points)))
    draw_image_lines(image, points, pointline, color=line_color, thickness=thickness)
    thickness_ = max(int(thickness * 1), 1)
    image = draw_points_text(image, points,
                             texts=texts,
                             color=circle_color,
                             thickness=thickness_,
                             drawType="simple")
    return image


def draw_image_points_arrowed_lines(image, points,
                                    pointline=[],
                                    texts=None,
                                    drawType="simple",
                                    reverse=False):
    '''
    在图像中画点箭头线
    :param img:
    :param points: 点列表
    :param pointline: `auto`->pointline = circle_line(len(points), iscircle=True)
    :param color:
    :param texts:
    :param drawType: simple or custom
    :param check:
    :return:
    '''
    points = np.asarray(points, dtype=np.int32)
    thickness = 2
    if texts is None:
        texts = list(range(len(points)))
    image = draw_points_text(image, points, texts=texts, thickness=thickness, drawType=drawType)
    draw_image_arrowed_lines(image, points, pointline, thickness=thickness, reverse=reverse)
    return image


def draw_image_lines(image, points, pointline=[], color=(0, 0, 255), thickness=2, check=True):
    points = np.asarray(points, dtype=np.int32)
    if pointline == "auto" or pointline == []:
        pointline = circle_line(len(points), iscircle=True)
    for point_index in pointline:
        point1 = tuple(points[point_index[0]])
        point2 = tuple(points[point_index[1]])
        if check:
            if point1 is None or point2 is None:
                continue
            if sum(point1) <= 0 or sum(point2) <= 0:
                continue
        cv2.line(image, point1, point2, color, thickness)  # 绿色，3个像素宽度
    return image


def draw_image_arrowed_lines(image,
                             points,
                             pointline=[],
                             color=(0, 0, 255),
                             thickness=2,
                             check=True,
                             reverse=False):
    points = np.asarray(points, dtype=np.int32)
    if pointline == "auto":
        pointline = circle_line(len(points), iscircle=True)
    for point_index in pointline:
        point1 = tuple(points[point_index[0]])
        point2 = tuple(points[point_index[1]])
        if check:
            if point1 is None or point2 is None:
                continue
            if sum(point1) <= 0 or sum(point2) <= 0:
                continue
        if reverse:
            cv2.arrowedLine(image, point1, point2, color, thickness=thickness)
        else:
            cv2.arrowedLine(image, point2, point1, color, thickness=thickness)
    return image


def draw_image_polylines(image, points, color=(0, 0, 255)):
    '''
    # points是三维坐标，分别表示(多边形个数，多边形坐标点x,多边形坐标点y)
    points = np.asarray([[[100, 100], [200, 100], [400, 200]],
                         [[500, 600], [300, 400], [500, 700]]]
                         )
    cv2.polylines(image,points,True,(255,0,0))  #画任意多边形
    参数2 pts：多边形的顶点坐标(按顺序)，points是三维坐标，分别表示(多边形个数，多边形坐标点x,多边形坐标点y)
    参数3 isClosed：必选参数。用于设置绘制的折线是否关闭，若设置为True，则从折线的最后一个顶点到其第一个顶点会自动绘制一条线进行闭合。
    参数4 color：必选参数。用于设置多边形的颜色
    参数5 lineType：可选参数。用于设置线段的类型，可选8（8邻接连接线-默认）、4（4邻接连接线）和cv2.LINE_AA 为抗锯齿
    '''
    image = cv2.polylines(image, points, isClosed=True, color=color)  # 画任意多边形
    return image


def circle_line(num_point, iscircle=True):
    '''
    产生连接线的点,用于绘制连接线
    points_line=circle_line(len(points),iscircle=True)
    >> [(0, 1), (1, 2), (2, 0)]
    :param num_point:
    :param iscircle: 首尾是否相连
    :return:
    '''
    start = 0
    end = num_point - 1
    points_line = []
    for i in range(start, end + 1):
        if i == end and iscircle:
            points_line.append([end, start])
        elif i != end:
            points_line.append([i, i + 1])
    return points_line


def cv_paste_image(im, mask, start_point=(0, 0)):
    """
    :param im:
    :param start_point:
    :param mask:
    :return:
    """
    xim, ymin = start_point
    shape = mask.shape  # h, w, d
    im[ymin:(ymin + shape[0]), xim:(xim + shape[1])] = mask
    return im


def pil_paste_image(im, mask, start_point=(0, 0)):
    """
    :param im:
    :param mask:
    :param start_point:
    :return:
    """
    out = Image.fromarray(im)
    mask = Image.fromarray(mask)
    out.paste(mask, start_point.copy())  # fix a bug
    return np.asarray(out)


def cv_rotate(image, angle, center=None, scale=1.0):  # 1
    (h, w) = image.shape[:2]  # 2
    if center is None:  # 3
        center = (w // 2, h // 2)  # 4
    M = cv2.getRotationMatrix2D(center, angle, scale)  # 5
    rotated = cv2.warpAffine(image, M, (w, h))  # 6
    return rotated  # 7


def rgb_to_gray(image):
    '''
    RGB to Gray image
    :param image:
    :return:
    '''
    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    return image


def convert_color_space(image, colorSpace='RGB'):
    if colorSpace == 'RGB':
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    elif colorSpace == 'BGR':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif colorSpace == 'GRAY':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    elif colorSpace == 'COLOR':
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        raise Exception("colorSpace error:{}".format(colorSpace))
    return image


def save_image(image_path, rgb_image, toUINT8=False):
    '''
    保存图片
    :param image_path:
    :param rgb_image:
    :param toUINT8:
    :return:
    '''
    save_dir = os.path.dirname(image_path)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    if toUINT8:
        rgb_image = np.asanyarray(rgb_image * 255, dtype=np.uint8)
    if len(rgb_image.shape) == 2:  # 若是灰度图则转为三通道
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_GRAY2BGR)
    else:
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    cv2.imwrite(image_path, bgr_image)


def save_image_lable_dir(save_root, image_list, image_ids, index):
    for i, (image, id) in enumerate(zip(image_list, image_ids)):
        image_path = os.path.join(save_root, str(id))
        if not os.path.exists(image_path):
            os.makedirs(image_path)
        image_path = os.path.join(image_path, str(index) + "_" + str(i) + ".jpg")
        save_image(image_path, image, toUINT8=False)


def combime_save_image(orig_image, dest_image, out_dir, name, prefix):
    '''
    命名标准：out_dir/name_prefix.jpg
    :param orig_image:
    :param dest_image:
    :param image_path:
    :param out_dir:
    :param prefix:
    :return:
    '''
    dest_path = os.path.join(out_dir, name + "_" + prefix + ".jpg")
    save_image(dest_path, dest_image)

    dest_image = np.hstack((orig_image, dest_image))
    save_image(os.path.join(out_dir, "{}_src_{}.jpg".format(name, prefix)), dest_image)


def combile_label_prob(label_list, prob_list):
    '''
    将label_list和prob_list拼接在一起，以便显示
    :param label_list:
    :param prob_list:
    :return:
    '''
    info = [str(l) + ":" + str(p)[:5] for l, p in zip(label_list, prob_list)]
    return info


def nms_bboxes_cv2(bboxes_list, scores_list, labels_list, width=None, height=None, score_threshold=0.5,
                   nms_threshold=0.45):
    '''
    NMS
    fix a bug: cv2.dnn.NMSBoxe bboxes, scores params must be list and float data,can not be float32 or int
    :param bboxes_list: [list[xmin,ymin,xmax,ymax],[],,,]
    :param scores_list: [float,...]
    :param labels_list: [int,...]
    :param width:
    :param height:
    :param score_threshold:
    :param nms_threshold:
    :return:
    '''
    assert isinstance(scores_list, list), "scores_list must be list"
    assert isinstance(bboxes_list, list), "bboxes_list must be list"
    assert isinstance(labels_list, list), "labels_list must be list"

    dest_bboxes_list = []
    dest_scores_list = []
    dest_labels_list = []
    # bboxes_list,scores_list, labels_list=filtering_scores(bboxes_list, scores_list, labels_list, score_threshold=score_threshold)
    if width is not None and height is not None:
        for i, box in enumerate(bboxes_list):
            x1 = box[0] * width
            y1 = box[1] * height
            x2 = box[2] * width
            y2 = box[3] * height
            bboxes_list[i] = [x1, y1, x2, y2]
    scores_list = np.asarray(scores_list, dtype=np.float).tolist()
    # fix a bug: cv2.dnn.NMSBoxe bboxes, scores params must be list and float data,can not be float32 or int
    indices = cv2.dnn.NMSBoxes(bboxes_list, scores_list, score_threshold, nms_threshold)
    for i in indices:
        i = i[0]
        dest_bboxes_list.append(bboxes_list[i])
        dest_scores_list.append(scores_list[i])
        dest_labels_list.append(labels_list[i])
    return dest_bboxes_list, dest_scores_list, dest_labels_list


def filtering_scores(bboxes_list, scores_list, labels_list, score_threshold=0.0):
    '''
    filtering low score bbox
    :param bboxes_list:
    :param scores_list:
    :param labels_list:
    :param score_threshold:
    :return:
    '''
    dest_scores_list = []
    dest_labels_list = []
    dest_bboxes_list = []
    for i, score in enumerate(scores_list):
        if score < score_threshold:
            continue
        dest_scores_list.append(scores_list[i])
        dest_labels_list.append(labels_list[i])
        dest_bboxes_list.append(bboxes_list[i])
    return dest_bboxes_list, dest_scores_list, dest_labels_list


def file2base64(file):
    image_base64 = base64.b64encode(open(file, 'rb').read()).decode()
    return image_base64


def image_to_base64(bgr_image):
    from io import BytesIO
    # image = cv2.imencode('.jpeg', bgr_image)[1]
    # image_base64 = base64.b64encode(image).decode()
    # image_base64 = str(base64.b64encode(image), encoding='utf-8')
    image = Image.fromarray(cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB))
    buff = BytesIO()
    image.save(buff, format="JPEG")
    image_base64 = base64.b64encode(buff.getvalue()).decode("utf-8")
    return image_base64


def base64_to_image(image_base64):
    # base64解码
    img_data = base64.b64decode(image_base64)
    # 转换为np数组
    rgb_array = np.fromstring(img_data, np.uint8)
    # 转换成opencv可用格式
    img = cv2.imdecode(rgb_array, cv2.IMREAD_COLOR)
    # img = cv2.imdecode(rgb_array, cv2.COLOR_BGR2RGB)
    return img


def read_image_base64(image_path, resize_height=None, resize_width=None):
    if resize_height is None and resize_width is None:
        with open(image_path, 'rb') as f_in:
            image_base64 = base64.b64encode(f_in.read())
            image_base64 = str(image_base64, encoding='utf-8')
    else:
        rgb_image = read_image(image_path, resize_height, resize_width)
        image_base64 = image_to_base64(rgb_image)
    return image_base64


def bin2image(bin_data, resize_height=None, resize_width=None, normalization=False, colorSpace='RGB'):
    data = np.asarray(bytearray(bin_data), dtype="uint8")
    bgr_image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if colorSpace == 'RGB':
        image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)  # 将BGR转为RGB
    elif colorSpace == "BGR":
        image = bgr_image
    else:
        exit(0)
    # show_image(filename,image)
    # image=Image.open(filename)
    image = resize_image(image, resize_height, resize_width)
    image = np.asanyarray(image)
    if normalization:
        image = image_normalization(image)
    # show_image("src resize image",image)
    return image


def post_process(input, axis=1):
    '''
    l2_norm
    :param input:
    :param axis:
    :return:
    '''
    # norm = torch.norm(input, 2, axis, True)
    # output = torch.div(input, norm)
    output = input / np.linalg.norm(input, axis=1, keepdims=True)
    return output


def softmax(x, axis=1):
    # 计算每行的最大值
    row_max = x.max(axis=axis)

    # 每行元素都需要减去对应的最大值，否则求exp(x)会溢出，导致inf情况
    row_max = row_max.reshape(-1, 1)
    x = x - row_max

    # 计算e的指数次幂
    x_exp = np.exp(x)
    x_sum = np.sum(x_exp, axis=axis, keepdims=True)
    s = x_exp / x_sum
    return s


def convert_anchor(anchors, height, width):
    '''
    height, width, _ = img.shape
    :param win_name:
    :param img:
    :param anchors: <class 'tuple'>: (nums, 4)
    :return: boxes_list:[xmin, ymin, xmax, ymax]
    '''
    boxes_list = []
    for index, anchor in enumerate(anchors):
        xmin = anchor[0] * width
        ymin = anchor[1] * height
        xmax = anchor[2] * width
        ymax = anchor[3] * height
        boxes_list.append([xmin, ymin, xmax, ymax])
    return boxes_list


def get_rect_crop_padding(image, rect):
    """
    :param image:
    :param rect:
    :return:
    """
    rect = [int(v) for v in rect]
    rows, cols, d = image.shape  # h,w,d
    x, y, width, height = rect
    crop_x1 = max(0, x)
    crop_y1 = max(0, y)
    crop_x2 = min(cols, x + width)  # 图像范围
    crop_y2 = min(rows, y + height)
    left_x = -x
    top_y = -y
    right_x = x + width - cols
    down_y = y + height - rows
    roi_image = image[crop_y1:crop_y2, crop_x1:crop_x2]
    # 只要存在边界越界的情况，就需要边界填充
    if top_y > 0 or down_y > 0 or left_x > 0 or right_x > 0:
        left_x = np.where(left_x > 0, left_x, 0)
        right_x = np.where(right_x > 0, right_x, 0)
        top_y = np.where(top_y > 0, top_y, 0)
        down_y = np.where(down_y > 0, down_y, 0)
        roi_image = cv2.copyMakeBorder(roi_image, int(top_y), int(down_y), int(left_x), int(right_x),
                                       cv2.BORDER_CONSTANT, value=0)
    return roi_image


def get_bbox_crop_padding(image, bbox):
    """
    :param image:
    :param bbox:
    :return:
    """
    rect = bboxes2rects([bbox])[0]
    roi_image = get_rect_crop_padding(image, rect)
    return roi_image


def get_bboxes_crop_padding(image, bboxes, resize_height=None, resize_width=None):
    """
    :param image:
    :param bboxes:
    :param resize:
    :return:
    """
    rects = bboxes2rects(bboxes)
    roi_images = []
    for rect in rects:
        roi_image = get_rect_crop_padding(image, rect)
        roi_image = resize_image(roi_image, resize_height, resize_width)
        roi_images.append(roi_image)
    return roi_images


def get_rects_crop_padding(image, rects, resize_height=None, resize_width=None):
    """
    :param image:
    :param rects:
    :param resize:
    :return:
    """
    roi_images = []
    for rect in rects:
        roi_image = get_rect_crop_padding(image, rect)
        roi_image = resize_image(roi_image, resize_height, resize_width)
        roi_images.append(roi_image)
    return roi_images


def center_crop(image, crop_size=[112, 112]):
    '''
    central_crop
    :param image: input numpy type image
    :param crop_size:crop_size must less than x.shape[:2]=[crop_h,crop_w]
    :return:
    '''
    h, w = image.shape[:2]
    y = int(round((h - crop_size[0]) / 2.))
    x = int(round((w - crop_size[1]) / 2.))
    y = max(y, 0)
    x = max(x, 0)
    return image[y:y + crop_size[0], x:x + crop_size[1]]


def center_crop_padding(image, crop_size):
    """
    :param image:
    :param crop_size: [crop_h,crop_w]
    :return:
    """
    h, w = image.shape[:2]
    y = int(round((h - crop_size[0]) / 2.))
    x = int(round((w - crop_size[1]) / 2.))
    rect = [x, y, crop_size[1], crop_size[0]]
    roi_image = get_rect_crop_padding(image, rect)
    return roi_image


def extend_face2body_bboxes(faces_boxes, width_factor=1.0, height_factor=1.0):
    '''
    extend boxes ,such as extend faces_boxes to body_boxes
    :param faces_boxes:
    :param width_factor:
    :param height_factor:
    :return:
    '''
    body_boxes = []
    for face_box in faces_boxes:
        [x1, y1, x2, y2] = face_box
        w = (x2 - x1)
        h = (y2 - y1)
        x1 = x1 - width_factor * w
        y1 = y1 - width_factor * h
        x1 = np.where(x1 > 0, x1, 0)
        y1 = np.where(y1 > 0, y1, 0)
        w = 3 * width_factor * w
        h = height_factor * h
        body_boxes.append([x1, y1, x1 + w, y1 + h])
    return body_boxes


def extend_rects(rects, scale=[1.0, 1.0]):
    """
    :param rects:
    :param scale: [sx,sy]==>(W,H)
    :return:
    """
    bboxes = rects2bboxes(rects)
    out_bboxes = extend_bboxes(bboxes, scale)
    out_rects = bboxes2rects(out_bboxes)
    return out_rects


def extend_bboxes(bboxes, scale=[1.0, 1.0]):
    """
    :param bboxes: [[xmin, ymin, xmax, ymax]]
    :param scale: [sx,sy]==>(W,H)
    :return:
    """
    out_bboxes = []
    sx = scale[0]
    sy = scale[1]
    for box in bboxes:
        xmin, ymin, xmax, ymax = box
        cx = (xmin + xmax) / 2
        cy = (ymin + ymax) / 2

        ex_w = (xmax - xmin) * sx
        ex_h = (ymax - ymin) * sy
        ex_xmin = cx - 0.5 * ex_w
        ex_ymin = cy - 0.5 * ex_h
        ex_xmax = ex_xmin + ex_w
        ex_ymax = ex_ymin + ex_h
        ex_box = [ex_xmin, ex_ymin, ex_xmax, ex_ymax]
        out_bboxes.append(ex_box)
    return out_bboxes


def get_square_bboxes(bboxes, fixed="H"):
    '''
    :param bboxes:
    :param fixed: (W)width (H)height,(L) longest edge
    :return:
    '''
    new_bboxes = []
    for bbox in bboxes:
        xmin, ymin, xmax, ymax = bbox
        w = xmax - xmin
        h = ymax - ymin
        cx, cy = (int((xmin + xmax) / 2), int((ymin + ymax) / 2))
        if fixed in ["H", "h"]:
            dd = h / 2
        elif fixed in ["W", "w"]:
            dd = w / 2
        elif fixed in ["L", "l"]:
            l = max(w, h)
            dd = l / 2
        else:
            l = max(w, h)
            dd = l / 2 * fixed
        fxmin = int(cx - dd)
        fymin = int(cy - dd)
        fxmax = int(cx + dd)
        fymax = int(cy + dd)
        new_bbox = (fxmin, fymin, fxmax, fymax)
        new_bboxes.append(new_bbox)
    return new_bboxes


def get_square_rects(rects, fixed="H"):
    """
    ------------------------
    e.g.:
    image_path = "../dataset/dataset/A/test1.jpg"
    image = read_image(image_path, 400, 400)
    rects = [[100, 50, 180, 50]]
    print(rects)
    rect_image = show_image_rects("rect", image, rects, waitKey=1)
    rects2 = get_square_rects(rects, fixed="h")
    # red is square_rects
    rect_image = show_image_rects("rect", rect_image, rects2, color=(255, 0, 0), waitKey=0)
    ------------------------
    :param rects:
    :param fixed:
    :return:
    """
    bboxes = rects2bboxes(rects)
    out_bboxes = get_square_bboxes(bboxes, fixed)
    out_rects = bboxes2rects(out_bboxes)
    return out_rects


def points2bbox(keypoints):
    joints_bbox = []
    for joints in keypoints:
        joints = np.asarray(joints)
        shape = joints.shape
        if len(shape) == 1:
            joints = joints.reshape(-1, 2)
        xmin = min(joints[:, 0])
        ymin = min(joints[:, 1])
        xmax = max(joints[:, 0])
        ymax = max(joints[:, 1])
        joints_bbox.append([xmin, ymin, xmax, ymax])
    return joints_bbox


def draw_yaws_pitchs_rolls_axis_in_image(image,
                                         yaws,
                                         pitchs,
                                         rolls,
                                         centers=None,
                                         size=75):
    """
    绘制yaw, pitch, roll坐标轴
    :param image:
    :param yaws:
    :param pitchs:
    :param rolls:
    :param centers:
    :param size:
    :return:
    """
    assert len(yaws) == len(pitchs)
    assert len(pitchs) == len(rolls)
    for i in range(len(yaws)):
        center = None if centers is None else centers[i]
        image = draw_yaw_pitch_roll_in_left_axis(image, yaws[i], pitchs[i], rolls[i], center, size=size)
    return image


def draw_yaw_pitch_roll_in_right_axis(image,
                                      yaw,
                                      pitch,
                                      roll,
                                      center=None,
                                      vis=True,
                                      size=75):
    """
    右手笛卡尔坐标：https://blog.csdn.net/a812073479/article/details/100103442
    pitch是围绕X轴旋转，也叫做俯仰角，左右耳朵连线构成X轴，如低头，点头动作
    yaw是围绕Y轴旋转，也叫偏航角，嘴巴鼻子连线构成Y轴，如左右摇头动作
    roll是围绕Z轴旋转，也叫翻滚角，眼睛和后脑勺连线构成Z轴，如歪头动作
    XYZ分别用红绿蓝表示，如(-10,0,0)表示绕X轴的反方向旋转10度
    :param image: BGR Image
    :param yaw:  绿色Y
    :param pitch: 红色X
    :param roll: 蓝色Z
    :param center:
    :param vis:
    :param size:
    :return:
    """
    text = "(pitch,yaw,roll)=({:3.3f},{:3.3f},{:3.3f})".format(pitch, yaw, roll)
    if center is None:
        h, w, c = image.shape
        center = (w / 2, h / 2)
    cx, cy = center
    pitch = pitch * np.pi / 180
    yaw = -(yaw * np.pi / 180)
    roll = roll * np.pi / 180
    # X-Axis pointing to right. drawn in red
    x1 = cx - size * (cos(yaw) * cos(roll))
    y1 = cy - size * (cos(pitch) * sin(roll) + cos(roll) * sin(pitch) * sin(yaw))
    color_yaw_x = (0, 0, 255)  # BGR
    # Y-Axis | drawn in green
    #        v
    x2 = size * (-cos(yaw) * sin(roll)) + cx
    y2 = size * (cos(pitch) * cos(roll) - sin(pitch) * sin(yaw) * sin(roll)) + cy
    color_pitch_y = (0, 255, 0)
    # Z-Axis (out of the screen) drawn in blue
    x3 = size * (sin(yaw)) + cx
    y3 = size * (-cos(yaw) * sin(pitch)) + cy
    color_roll_z = (255, 0, 0)
    cv2.arrowedLine(image, (int(cx), int(cy)), (int(x1), int(y1)), color_yaw_x, 2, tipLength=0.2)
    cv2.arrowedLine(image, (int(cx), int(cy)), (int(x2), int(y2)), color_pitch_y, 2, tipLength=0.2)
    cv2.arrowedLine(image, (int(cx), int(cy)), (int(x3), int(y3)), color_roll_z, 2, tipLength=0.2)
    if vis:
        fontFace = cv2.FONT_HERSHEY_SIMPLEX
        image = cv2.putText(image, str(text),
                            # (int(cx), int(cy) + 10),
                            (10, 10),
                            fontFace,
                            fontScale=0.3,
                            color=(255, 0, 0),
                            thickness=1)
    return image


def draw_yaw_pitch_roll_in_left_axis(image,
                                     yaw,
                                     pitch,
                                     roll,
                                     center=None,
                                     vis=True,
                                     size=75):
    """
    左手坐标
    """
    text = "(pitch,yaw,roll)=({:3.3f},{:3.3f},{:3.3f})".format(pitch, yaw, roll)
    if center is None:
        h, w, c = image.shape
        center = (w / 2, h / 2)
    cx, cy = center
    pitch = pitch * np.pi / 180
    yaw = -(yaw * np.pi / 180)
    roll = roll * np.pi / 180
    # X-Axis pointing to right. drawn in red
    x1 = size * (cos(yaw) * cos(roll)) + cx
    y1 = size * (cos(pitch) * sin(roll) + cos(roll) * sin(pitch) * sin(yaw)) + cy
    color_yaw_x = (0, 0, 255)  # BGR
    # Y-Axis | drawn in green
    #        v
    x2 = size * (-cos(yaw) * sin(roll)) + cx
    y2 = size * (cos(pitch) * cos(roll) - sin(pitch) * sin(yaw) * sin(roll)) + cy
    color_pitch_y = (0, 255, 0)
    # Z-Axis (out of the screen) drawn in blue
    x3 = size * (sin(yaw)) + cx
    y3 = size * (-cos(yaw) * sin(pitch)) + cy
    color_roll_z = (255, 0, 0)
    cv2.arrowedLine(image, (int(cx), int(cy)), (int(x1), int(y1)), color_yaw_x, 2, tipLength=0.2)
    cv2.arrowedLine(image, (int(cx), int(cy)), (int(x2), int(y2)), color_pitch_y, 2, tipLength=0.2)
    cv2.arrowedLine(image, (int(cx), int(cy)), (int(x3), int(y3)), color_roll_z, 2, tipLength=0.2)
    if vis:
        fontFace = cv2.FONT_HERSHEY_SIMPLEX
        image = cv2.putText(image, str(text),
                            # (int(cx), int(cy) + 10),
                            (10, 10),
                            fontFace,
                            fontScale=0.3,
                            color=(255, 0, 0),
                            thickness=1)
    return image


def fig2data(fig):
    """
    fig = plt.figure()
    image = fig2data(fig)
    @brief Convert a Matplotlib figure to a 4D numpy array with RGBA channels and return it
    @param fig a matplotlib figure
    @return a numpy 3D array of RGBA values
    """
    # draw the renderer
    fig.canvas.draw()

    # Get the RGBA buffer from the figure
    w, h = fig.canvas.get_width_height()
    buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
    buf.shape = (w, h, 4)

    # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
    buf = np.roll(buf, 3, axis=2)
    image = Image.frombytes("RGBA", (w, h), buf.tostring())
    image = np.asarray(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
    return image


def addMouseCallback(winname, param, callbackFunc=None):
    '''
     添加点击事件
    :param winname:
    :param param:
    :param callbackFunc:
    :return:
    '''
    cv2.namedWindow(winname)

    def default_callbackFunc(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("(x,y)=({},{}),data={}".format(x, y, param[y][x]))

    if callbackFunc is None:
        callbackFunc = default_callbackFunc
    cv2.setMouseCallback(winname, callbackFunc, param)


class EventCv():
    def __init__(self):
        self.image = None

    def update_image(self, image):
        self.image = image

    def callback_print_image(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("(x,y)=({},{}),data={}".format(x, y, self.image[y][x]))

    def add_mouse_event(self, winname, param=None, callbackFunc=None):
        '''
         添加点击事件
        :param winname:
        :param param:
        :param callbackFunc:
        :return:
        '''
        cv2.namedWindow(winname)
        if callbackFunc is None:
            callbackFunc = self.callback_print_image
        cv2.setMouseCallback(winname, callbackFunc, param=param)


def get_video_capture(video_path, width=None, height=None, fps=None):
    """
     --   7W   Pix--> width=320,height=240
     --   30W  Pix--> width=640,height=480
     720P,100W Pix--> width=1280,height=720
     960P,130W Pix--> width=1280,height=1024
    1080P,200W Pix--> width=1920,height=1080
    :param video_path:
    :param width:
    :param height:
    :return:
    """
    video_cap = cv2.VideoCapture(video_path)
    # 设置分辨率
    if width:
        video_cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        video_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps:
        video_cap.set(cv2.CAP_PROP_FPS, 15)
    return video_cap


def get_image_block(image, grid=[3, 3], same=False):
    """
    :param image:
    :param grid: cell grid
    :param same: True : 保证每个cell的大小相同,会裁剪掉部分像素
           same: False: 及可能每个cell的大小相同,边界的cell会偏大
    :return:
    """
    image_block = []
    grid_w, grid_h = grid
    height, width, channel = np.shape(image)  # H,W,C
    step_w = int(height / grid_w)
    step_h = int(width / grid_h)
    for i in range(0, grid_h):
        for j in range(0, grid_w):
            x1 = j * step_w
            y1 = i * step_h
            x2 = (j + 1) * step_w
            y2 = (i + 1) * step_h
            if same:
                block = image[y1:y2, x1:x2]
            else:
                x2 = x2 if x2 + step_w < width else width
                y2 = y2 if y2 + step_h < height else height
                block = image[y1:y2, x1:x2]
            cv_show_image("block", block)
            image_block.append(block)
    return image_block


def image_list2gif(image_list, image_size=None, out_gif_path="test.gif", fps=2):
    """
    uri：合成后的gif动图的名字，可以随意更改。
    mode：操作模式，I表示多图，不用更改。
    fps：帧率，也就是画面每秒传输帧数，值越大，gif动图的播放速度越大。
    Usge:
        image_dir="path/to/image-directory"
        image_list = file_processing.get_files_lists(image_dir)
        image_list2gif(image_list, out_gif_path=out_gif_path, fps=fps)
    :param image_list:图片列表
    :param image_size:gif图片的大小
    :param out_gif_path:
    :param fps:
    :return:
    """

    import imageio
    # pip install imageio

    if not image_size:
        image_size = [None, None]
    if isinstance(image_size, numbers.Number):
        image_size = [image_size, image_size]
    writer = imageio.get_writer(uri=out_gif_path, mode='I', fps=fps)
    for image_path in image_list:
        # image = imageio.imread(image_path)
        rgb = cv2.imread(image_path)
        image = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        image = resize_image(image, resize_height=image_size[1], resize_width=image_size[0])
        writer.append_data(image)
    writer.close()
    print("save gif:{}".format(out_gif_path))


def get_video_info(video_cap):
    width = int(video_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    numFrames = int(video_cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = int(video_cap.get(cv2.CAP_PROP_FPS))
    print("video:width:{},height:{},fps:{},numFrames:{}".format(width, height, fps, numFrames))
    return width, height, numFrames, fps


def get_video_writer(save_path, width, height, fps):
    if not os.path.exists(os.path.dirname(save_path)):
        os.makedirs(os.path.dirname(save_path))
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    frameSize = (int(width), int(height))
    video_writer = cv2.VideoWriter(save_path, fourcc, fps, frameSize)
    print("video:width:{},height:{},fps:{}".format(width, height, fps))
    return video_writer


class CVVideo():
    def __init__(self):
        pass

    def start_capture(self, video_path, save_video=None, detect_freq=1):
        """
        start capture video
        :param video_path: *.avi,*.mp4,...
        :param save_video: *.avi
        :param detect_freq:
        :return:
        """
        # cv2.moveWindow("test", 1000, 100)
        video_cap = get_video_capture(video_path)
        width, height, numFrames, fps = get_video_info(video_cap)
        if save_video:
            self.video_writer = get_video_writer(save_video, width, height, fps)
        # freq = int(fps / detect_freq)
        count = 0
        while True:
            isSuccess, frame = video_cap.read()
            # frame = video_cap.set(cv2.CAP_PROP_POS_FRAMES, count)
            if not isSuccess:
                break
            if count % detect_freq == 0:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = self.task(frame)
            if save_video:
                self.write_video(frame)
            count += 1
        video_cap.release()

    def write_video(self, frame):
        self.video_writer.write(frame)

    def task(self, frame):
        # TODO
        cv2.imshow("image", frame)
        cv2.moveWindow("image", 0, 0)
        cv2.waitKey(10)
        return frame


if __name__ == "__main__":
    from utils import image_utils

    input_size = [int(490 / 2), int(800 / 2)]
    image_path = "test.jpg"
    src_boxes = [[8.20251, 1, 242.2412, 699.2236],
                 [201.14865, 204.18265, 468.605, 696.36163]]
    src_boxes = np.asarray(src_boxes)
    image = read_image(image_path)  # (800, 490, 3)
    image1, boxes1 = image_boxes_resize_padding(image, input_size, src_boxes)
    image1 = show_image_boxes("Det", image1, boxes1, color=(255, 0, 0), waitKey=3)
    boxes = image_boxes_resize_padding_inverse((image.shape[1], image.shape[0]), input_size, boxes1)
    show_image_boxes("image", image, boxes)
