from unittest import TestCase

import numpy as np

from kmeans import iou, avg_iou, kmeans


class TestBasic(TestCase):
    def test_iou_100(self):
        self.assertEqual(iou([200, 200], np.array([[200, 200]])), 1.)

    def test_iou_50(self):
        self.assertEqual(iou([200, 200], np.array([[100, 200]])), .5)
        self.assertEqual(iou([200, 200], np.array([[200, 100]])), .5)

    def test_iou_75(self):
        self.assertEqual(iou([200, 200], np.array([[150, 200]])), .75)
        self.assertEqual(iou([200, 200], np.array([[200, 150]])), .75)

    def test_iou_20(self):
        self.assertEqual(iou([183, 73], np.array([[73, 36.6]])), .2)
        self.assertEqual(iou([183, 73], np.array([[36.6, 73]])), .2)

    def test_iou_multiple(self):
        a = np.array([[200, 200], [100, 200], [200, 100], [150, 200], [200, 150]])
        b = np.array([1., 0.5, 0.5, 0.75, 0.75])
        self.assertTrue((iou([200, 200], a) == b).all())

    def test_iou_0(self):
        self.assertRaises(ValueError, iou, [100, 100], np.array([[0, 0]]))
        self.assertRaises(ValueError, iou, [0, 0], np.array([[100, 100]]))
        self.assertRaises(ValueError, iou, [0, 0], np.array([[0, 0]]))
        self.assertRaises(ValueError, iou, [100, 0], np.array([[100, 100]]))
        self.assertRaises(ValueError, iou, [0, 100], np.array([[100, 100]]))

    def test_avg_iou_simple(self):
        self.assertEqual(avg_iou(np.array([[200, 200]]), np.array([[200, 200]])), 1.)
        self.assertEqual(avg_iou(np.array([[200, 200]]), np.array([[100, 200]])), .5)
        self.assertEqual(avg_iou(np.array([[200, 200]]), np.array([[200, 100]])), .5)

    def test_avg_iou_multiple(self):
        a = np.array([[200, 200], [100, 200], [200, 100], [150, 200], [200, 150]])
        b = np.array([[200, 200], [100, 200], [200, 100], [150, 200], [200, 150]])
        self.assertEqual(avg_iou(a, b), 1.)

        c = np.array([[200, 200], [100, 200]])
        self.assertEqual(avg_iou(a, c), np.mean([1., 1., .5, .75, .75]))

    def test_kmeans_simple(self):
        a = np.array([[200, 200]])
        b = np.array([[200, 200]])
        self.assertTrue((kmeans(a, 1) == b).all())

    def test_kmeans_multiple(self):
        a = np.array([[200, 200], [100, 200], [300, 200]])
        b = [[100, 200], [200, 200], [300, 200]]

        out = kmeans(a, 3).tolist()
        out.sort()

        self.assertTrue((out == b))
