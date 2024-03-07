# MIT License

# Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.



"""
Prototype of Utility functions and GJK algorithm for Collision checks between vehicles
Originally from https://github.com/kroitor/gjk.c
Author: Hongrui Zheng
"""

import numpy as np
from numba import njit

@njit(cache=True)
def perpendicular(pt):
    """
    Return a 2-vector's perpendicular vector

    Args:
        pt (np.ndarray, (2,)): input vector

    Returns:
        pt (np.ndarray, (2,)): perpendicular vector
    """
    temp = pt[0]
    pt[0] = pt[1]
    pt[1] = -1*temp
    return pt


@njit(cache=True)
def tripleProduct(a, b, c):
    """
    Return triple product of three vectors

    Args:
        a, b, c (np.ndarray, (2,)): input vectors

    Returns:
        (np.ndarray, (2,)): triple product
    """
    ac = a.dot(c)
    bc = b.dot(c)
    return b*ac - a*bc


@njit(cache=True)
def avgPoint(vertices):
    """
    Return the average point of multiple vertices

    Args:
        vertices (np.ndarray, (n, 2)): the vertices we want to find avg on

    Returns:
        avg (np.ndarray, (2,)): average point of the vertices
    """
    return np.sum(vertices, axis=0)/vertices.shape[0]


@njit(cache=True)
def indexOfFurthestPoint(vertices, d):
    """
    Return the index of the vertex furthest away along a direction in the list of vertices

    Args:
        vertices (np.ndarray, (n, 2)): the vertices we want to find avg on

    Returns:
        idx (int): index of the furthest point
    """
    return np.argmax(vertices.dot(d))


@njit(cache=True)
def support(vertices1, vertices2, d):
    """
    Minkowski sum support function for GJK

    Args:
        vertices1 (np.ndarray, (n, 2)): vertices of the first body
        vertices2 (np.ndarray, (n, 2)): vertices of the second body
        d (np.ndarray, (2, )): direction to find the support along

    Returns:
        support (np.ndarray, (n, 2)): Minkowski sum
    """
    i = indexOfFurthestPoint(vertices1, d)
    j = indexOfFurthestPoint(vertices2, -d)
    return vertices1[i] - vertices2[j]


@njit(cache=True)
def collision(vertices1, vertices2):
    """
    GJK test to see whether two bodies overlap

    Args:
        vertices1 (np.ndarray, (n, 2)): vertices of the first body
        vertices2 (np.ndarray, (n, 2)): vertices of the second body

    Returns:
        overlap (boolean): True if two bodies collide
    """
    index = 0
    simplex = np.empty((3, 2))

    position1 = avgPoint(vertices1)
    position2 = avgPoint(vertices2)

    d = position1 - position2

    if d[0] == 0 and d[1] == 0:
        d[0] = 1.0

    a = support(vertices1, vertices2, d)
    simplex[index, :] = a

    if d.dot(a) <= 0:
        return False

    d = -a

    iter_count = 0
    while iter_count < 1e3:
        a = support(vertices1, vertices2, d)
        index += 1
        simplex[index, :] = a
        if d.dot(a) <= 0:
            return False

        ao = -a

        if index < 2:
            b = simplex[0, :]
            ab = b-a
            d = tripleProduct(ab, ao, ab)
            if np.linalg.norm(d) < 1e-10:
                d = perpendicular(ab)
            continue

        b = simplex[1, :]
        c = simplex[0, :]
        ab = b-a
        ac = c-a

        acperp = tripleProduct(ab, ac, ac)

        if acperp.dot(ao) >= 0:
            d = acperp
        else:
            abperp = tripleProduct(ac, ab, ab)
            if abperp.dot(ao) < 0:
                return True
            simplex[0, :] = simplex[1, :]
            d = abperp

        simplex[1, :] = simplex[2, :]
        index -= 1

        iter_count += 1
    return False

@njit(cache=True)
def collision_multiple(vertices):
    """
    Check pair-wise collisions for all provided vertices

    Args:
        vertices (np.ndarray (num_bodies, 4, 2)): all vertices for checking pair-wise collision

    Returns:
        collisions (np.ndarray (num_vertices, )): whether each body is in collision
        collision_idx (np.ndarray (num_vertices, )): which index of other body is each index's body is in collision, -1 if not in collision
    """
    collisions = np.zeros((vertices.shape[0], ))
    collision_idx = -1 * np.ones((vertices.shape[0], ))
    # looping over all pairs
    for i in range(vertices.shape[0]-1):
        for j in range(i+1, vertices.shape[0]):
            # check collision
            vi = np.ascontiguousarray(vertices[i, :, :])
            vj = np.ascontiguousarray(vertices[j, :, :])
            ij_collision = collision(vi, vj)
            # fill in results
            if ij_collision:
                collisions[i] = 1.
                collisions[j] = 1.
                collision_idx[i] = j
                collision_idx[j] = i

    return collisions, collision_idx

"""
Utility functions for getting vertices by pose and shape
"""

@njit(cache=True)
def get_trmtx(pose):
    """
    Get transformation matrix of vehicle frame -> global frame

    Args:
        pose (np.ndarray (3, )): current pose of the vehicle

    return:
        H (np.ndarray (4, 4)): transformation matrix
    """
    x = pose[0]
    y = pose[1]
    th = pose[2]
    cos = np.cos(th)
    sin = np.sin(th)
    H = np.array([[cos, -sin, 0., x], [sin, cos, 0., y], [0., 0., 1., 0.], [0., 0., 0., 1.]])
    return H

@njit(cache=True)
def get_vertices(pose, length, width):
    """
    Utility function to return vertices of the car body given pose and size

    Args:
        pose (np.ndarray, (3, )): current world coordinate pose of the vehicle
        length (float): car length
        width (float): car width

    Returns:
        vertices (np.ndarray, (4, 2)): corner vertices of the vehicle body
    """
    H = get_trmtx(pose)
    rl = H.dot(np.asarray([[-length/2],[width/2],[0.], [1.]])).flatten()
    rr = H.dot(np.asarray([[-length/2],[-width/2],[0.], [1.]])).flatten()
    fl = H.dot(np.asarray([[length/2],[width/2],[0.], [1.]])).flatten()
    fr = H.dot(np.asarray([[length/2],[-width/2],[0.], [1.]])).flatten()
    rl = rl/rl[3]
    rr = rr/rr[3]
    fl = fl/fl[3]
    fr = fr/fr[3]
    vertices = np.asarray([[rl[0], rl[1]], [rr[0], rr[1]], [fr[0], fr[1]], [fl[0], fl[1]]])
    return vertices


"""
Unit tests for GJK collision checks
Author: Hongrui Zheng
"""

import time
import unittest

class CollisionTests(unittest.TestCase):
    def setUp(self):
        # test params
        np.random.seed(1234)

        # Collision check body
        self.vertices1 = np.asarray([[4,11.],[5,5],[9,9],[10,10]])

        # car size
        self.length = 0.32
        self.width = 0.22
    
    def test_get_vert(self):
        test_pose = np.array([2.3, 6.7, 0.8])
        vertices = get_vertices(test_pose, self.length, self.width)
        rect = np.vstack((vertices, vertices[0,:]))
        import matplotlib.pyplot as plt
        plt.scatter(test_pose[0], test_pose[1], c='red')
        plt.plot(rect[:, 0], rect[:, 1])
        plt.xlim([1, 4])
        plt.ylim([5, 8])
        plt.axes().set_aspect('equal')
        plt.show()
        self.assertTrue(vertices.shape == (4, 2))

    def test_get_vert_fps(self):
        test_pose = np.array([2.3, 6.7, 0.8])
        start = time.time()
        for _ in range(1000):
            vertices = get_vertices(test_pose, self.length, self.width)
        elapsed = time.time() - start
        fps = 1000/elapsed
        print('get vertices fps:', fps)
        self.assertTrue(fps>500)

    def test_random_collision(self):
        # perturb the body by a small amount and make sure it all collides with the original body
        for _ in range(1000):
            a = self.vertices1 + np.random.normal(size=(self.vertices1.shape))/100.
            b = self.vertices1 + np.random.normal(size=(self.vertices1.shape))/100.
            self.assertTrue(collision(a,b))

    def test_multiple_collisions(self):
        a = self.vertices1 + np.random.normal(size=(self.vertices1.shape))/100.
        b = self.vertices1 + np.random.normal(size=(self.vertices1.shape))/100.
        c = self.vertices1 + np.random.normal(size=(self.vertices1.shape))/100.
        d = self.vertices1 + np.random.normal(size=(self.vertices1.shape))/100.
        e = self.vertices1 + np.random.normal(size=(self.vertices1.shape))/100.
        f = self.vertices1 + np.random.normal(size=(self.vertices1.shape))/100.
        g = self.vertices1 + 10.
        allv = np.stack((a,b,c,d,e,f,g))
        collisions, collision_idx = collision_multiple(allv)
        self.assertTrue(np.all(collisions == np.array([1., 1., 1., 1., 1., 1., 0.])))
        self.assertTrue(np.all(collision_idx == np.array([5., 5., 5., 5., 5., 4., -1.])))

    def test_fps(self):
        # also perturb the body but mainly want to test GJK speed
        start = time.time()
        for _ in range(1000):
            a = self.vertices1 + np.random.normal(size=(self.vertices1.shape))/100.
            b = self.vertices1 + np.random.normal(size=(self.vertices1.shape))/100.
            collision(a, b)
        elapsed = time.time() - start
        fps = 1000/elapsed
        print('gjk fps:', fps)
        self.assertTrue(fps>500)

if __name__ == '__main__':
    unittest.main()