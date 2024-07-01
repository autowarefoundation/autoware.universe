import glob
import xml.etree.ElementTree as ET
import numpy as np
from tqdm import tqdm
from pybaseutils import file_utils
from engine.kmeans_anchor.kmeans import kmeans, avg_iou


def load_voc_bboxes(path, class_names=None):
    bbox_wh = []
    classes_set = set()
    if isinstance(path, str): path = [path]
    xml_list = []
    for p in path: xml_list += file_utils.get_files_lists(p, postfix=["*.xml"])
    for xml_file in tqdm(xml_list):
        tree = ET.parse(xml_file)

        height = int(tree.findtext("./size/height"))
        width = int(tree.findtext("./size/width"))
        for obj in tree.iter("object"):
            difficult = obj.find('difficult').text
            name = obj.find('name').text
            if class_names and (name not in class_names):
                continue
            classes_set.add(name)
            xmin = float(obj.findtext("bndbox/xmin")) / width
            ymin = float(obj.findtext("bndbox/ymin")) / height
            xmax = float(obj.findtext("bndbox/xmax")) / width
            ymax = float(obj.findtext("bndbox/ymax")) / height
            w = xmax - xmin
            h = ymax - ymin
            if w <= 0 or h <= 0:
                print(xml_file)
                continue
            bbox_wh.append([w, h])
    print("classes_set:{}".format(classes_set))
    print("have bboxes:{}".format(len(bbox_wh)))
    bbox_wh = np.array(bbox_wh)
    bbox_wh = np.clip(bbox_wh, 0, 1)
    return bbox_wh


def kmean_for_anchor(bbox_wh, num_cluster, input_size=320):
    """
    accuracy: 76.29%
    anchor:
     [[0.02239583 0.04537037]
     [0.04779412 0.025     ]
     [0.03072917 0.05555556]
     [0.03854167 0.07222222]
     [0.08333333 0.04444444]
     [0.05520833 0.09907407]]
    anchor*320:
     [[ 7.16666667 14.51851852]
     [15.29411765  8.        ]
     [ 9.83333333 17.77777778]
     [12.33333333 23.11111111]
     [26.66666667 14.22222222]
     [17.66666667 31.7037037 ]]
    area:
     [0.00101611 0.00119485 0.00170718 0.00278356 0.0037037  0.00546971]
    ratios:
     [0.49, 1.91, 0.55, 0.53, 1.88, 0.56]
    :param bbox_wh:
    :param num_cluster:
    :return:
    """
    anchor = kmeans(bbox_wh, k=num_cluster, seed=2020)
    area = anchor[:, 0] * anchor[:, 1]
    index = np.argsort(area)
    anchor = anchor[index, :]
    area = area[index]
    acc = avg_iou(bbox_wh, anchor) * 100
    ratios = np.around(anchor[:, 0] / anchor[:, 1], decimals=2).tolist()
    print("accuracy: {:.2f}%".format(acc))
    print("anchor:\n {}".format(anchor))
    print("anchor*{}:\n {}".format(input_size, anchor * input_size))
    print("area:\n {}".format(area))
    print("ratios:\n {}".format(ratios))
    return anchor


if __name__ == "__main__":
    """
    anchor*640:
         [[ 21.87670615  19.77830448]
         [ 27.23432181  19.07222837]
         [ 29.50053551  21.80555556]
         [ 30.92590741  24.86111111]
         [ 35.37037037  26.58461538]
         [ 39.81170141  31.00288034]
         [ 87.3170439   64.79012346]
         [ 96.98388428 166.5585482 ]
         [584.07767612 584.34783227]]
         
    """
    # ann_dir = "/home/dm/panjinquan3/dataset/VOCdevkit/VOC2007/Annotations"
    ann_dir = [
        "/home/dm/nasdata/dataset-dmai/handwriting/word-det/word-supplement/Annotations",
        "/home/dm/nasdata/dataset-dmai/handwriting/word-det/word-lesson1-16/Annotations",
        "/home/dm/nasdata/dataset-dmai/handwriting/word-det/word-old/Annotations",
        "/home/dm/nasdata/dataset-dmai/handwriting/word-det/competition/Annotations"
    ]

    # class_names = ["fingernail"]
    class_names = None
    num_cluster = 9
    bbox_wh = load_voc_bboxes(ann_dir, class_names=class_names)
    kmean_for_anchor(bbox_wh, num_cluster, input_size=640)
