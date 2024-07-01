from unittest import TestCase

import numpy as np

from kmeans import kmeans


class TestBasic(TestCase):
    def gen_shape(self, width, height, amount):
        boxes = np.empty((amount, 2))
        for i in range(0, amount):
            x0 = np.random.randint(100, 1000)
            y0 = np.random.randint(100, 1000)
            x1 = x0 + width
            y1 = y0 + height
            boxes[i] = (x1 - x0, y1 - y0)
        return boxes

    def test_kmeans_shift(self):
        boxes = self.gen_shape(1000, 1000, 100)
        self.assertTrue((kmeans(boxes, 1) == [[1000, 1000]]).all())

    def test_kmeans_shift2(self):
        boxes1 = self.gen_shape(1000, 1000, 1)
        boxes2 = self.gen_shape(100, 3000, 1)
        together = np.concatenate((boxes1, boxes2), axis=0)

        out = kmeans(together, 2)

        res1 = np.array([[100, 3000], [1000, 1000]])
        res2 = np.array([[1000, 1000], [100, 3000]])
        self.assertTrue(np.array_equal(out, res1) or np.array_equal(out, res2))
