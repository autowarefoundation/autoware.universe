# yolov5-enhance

## 修改内容
- 增加scripts文件夹
- 增加file_utils.py和image_utils.py工具
- 增加支持VOC数据格式训练：parser_voc.py和voc_datasets.py,需求修改`train.py`和`utils/datasets.py`
```python
        # (1)train.py中create_dataloader中增加参数:data_dict
        # (2)utils/datasets.py选择不同的数据类型：data_type
        if "data_type" in data_dict and data_dict["data_type"] == "voc":
            from utils import voc_datasets
            dataset = voc_datasets.LoadVOCImagesAndLabels(path, imgsz, batch_size,
                                                          augment=augment,  # augment images
                                                          hyp=hyp,  # augmentation hyperparameters
                                                          rect=rect,  # rectangular training
                                                          cache_images=cache,
                                                          single_cls=single_cls,
                                                          stride=int(stride),
                                                          pad=pad,
                                                          image_weights=image_weights,
                                                          prefix=prefix,
                                                          names=data_dict["names"])
        else:
            dataset = LoadImagesAndLabels
```  
- dataset中显示img, labels
```python

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

```


## 自适应图片缩放(只是在测试推理使用)
- https://www.cnblogs.com/tian777/p/14987578.html
  
        在目标检测中，不同的图片长宽都不相同， 常用的方式是将原始图片resize() 缩放到一个标准尺寸, 针对长宽比较大的图片 Yolov5代码中对此进行了改进。
        在Yolov5代码中datasets.py的letterbox函数中进行了修改，对原始图像自适应的添加最少的黑边。
        主要有以下几步
        1、计算缩放比例 可以得到0.52，和0.69两个缩放系数，选择小的缩放系数0.52
        2、计算缩放后的尺寸 原始图片的长宽都乘以最小的缩放系数 得到长、宽
        3、计算黑边填充数值 得到原本需要填充的高度， 采用numpy np.mod(x,32)取余数的方式
        np.mod函数的后面用32 , 因为 Yolov5的网络经过5次下采样
        注意：
        1、训练时没有采用缩减黑边的方式，还是采用传统填充的方式
        2、只是在测试，使用模型推理时，才采用缩减黑边的方式，提高目标检测，推理的速度。