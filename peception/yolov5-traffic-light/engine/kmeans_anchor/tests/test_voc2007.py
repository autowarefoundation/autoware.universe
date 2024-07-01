import glob
import xml.etree.ElementTree as ET
from unittest import TestCase

import numpy as np

from kmeans import kmeans, avg_iou

ANNOTATIONS_PATH = "Annotations"


class TestVoc2007(TestCase):
    def __load_dataset(self):
        dataset = []
        for xml_file in glob.glob("{}/*xml".format(ANNOTATIONS_PATH)):
            tree = ET.parse(xml_file)

            height = int(tree.findtext("./size/height"))
            width = int(tree.findtext("./size/width"))

            for obj in tree.iter("object"):
                xmin = int(obj.findtext("bndbox/xmin")) / width
                ymin = int(obj.findtext("bndbox/ymin")) / height
                xmax = int(obj.findtext("bndbox/xmax")) / width
                ymax = int(obj.findtext("bndbox/ymax")) / height

                dataset.append([xmax - xmin, ymax - ymin])

        return np.array(dataset)

    def test_kmeans_5(self):
        dataset = self.__load_dataset()

        out = kmeans(dataset, 5)
        percentage = avg_iou(dataset, out)

        np.testing.assert_almost_equal(percentage, 0.61, decimal=2)

    def test_kmeans_9(self):
        dataset = self.__load_dataset()

        out = kmeans(dataset, 9)
        percentage = avg_iou(dataset, out)

        np.testing.assert_almost_equal(percentage, 0.672, decimal=2)
