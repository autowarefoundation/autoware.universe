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
Prototype of Utility functions and classes for simulating 2D LIDAR scans
Author: Hongrui Zheng
"""

import numpy as np
from numba import njit
from scipy.ndimage import distance_transform_edt as edt
from PIL import Image
import os
import yaml

import unittest
import timeit

def get_dt(bitmap, resolution):
    """
    Distance transformation, returns the distance matrix from the input bitmap.
    Uses scipy.ndimage, cannot be JITted.

        Args:
            bitmap (numpy.ndarray, (n, m)): input binary bitmap of the environment, where 0 is obstacles, and 255 (or anything > 0) is freespace
            resolution (float): resolution of the input bitmap (m/cell)

        Returns:
            dt (numpy.ndarray, (n, m)): output distance matrix, where each cell has the corresponding distance (in meters) to the closest obstacle
    """
    dt = resolution * edt(bitmap)
    return dt

@njit(cache=True)
def xy_2_rc(x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution):
    """
    Translate (x, y) coordinate into (r, c) in the matrix

        Args:
            x (float): coordinate in x (m)
            y (float): coordinate in y (m)
            orig_x (float): x coordinate of the map origin (m)
            orig_y (float): y coordinate of the map origin (m)
        
        Returns:
            r (int): row number in the transform matrix of the given point
            c (int): column number in the transform matrix of the given point
    """
    # translation
    x_trans = x - orig_x
    y_trans = y - orig_y

    # rotation
    x_rot = x_trans * orig_c + y_trans * orig_s
    y_rot = -x_trans * orig_s + y_trans * orig_c

    # clip the state to be a cell
    if x_rot < 0 or x_rot >= width * resolution or y_rot < 0 or y_rot >= height * resolution:
        c = -1
        r = -1
    else:
        c = int(x_rot/resolution)
        r = int(y_rot/resolution)

    return r, c

@njit(cache=True)
def distance_transform(x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt):
    """
    Look up corresponding distance in the distance matrix

        Args:
            x (float): x coordinate of the lookup point
            y (float): y coordinate of the lookup point
            orig_x (float): x coordinate of the map origin (m)
            orig_y (float): y coordinate of the map origin (m)

        Returns:
            distance (float): corresponding shortest distance to obstacle in meters
    """
    r, c = xy_2_rc(x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution)
    distance = dt[r, c]
    return distance

@njit(cache=True)
def trace_ray(x, y, theta_index, sines, cosines, eps, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt, max_range):
    """
    Find the length of a specific ray at a specific scan angle theta
    Purely math calculation and loops, should be JITted.

        Args:
            x (float): current x coordinate of the ego (scan) frame
            y (float): current y coordinate of the ego (scan) frame
            theta_index(int): current index of the scan beam in the scan range
            sines (numpy.ndarray (n, )): pre-calculated sines of the angle array
            cosines (numpy.ndarray (n, )): pre-calculated cosines ...

        Returns:
            total_distance (float): the distance to first obstacle on the current scan beam
    """
    
    # int casting, and index precal trigs
    theta_index_ = int(theta_index)
    s = sines[theta_index_]
    c = cosines[theta_index_]

    # distance to nearest initialization
    dist_to_nearest = distance_transform(x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt)
    total_dist = dist_to_nearest

    # ray tracing iterations
    while dist_to_nearest > eps and total_dist <= max_range:
        # move in the direction of the ray by dist_to_nearest
        x += dist_to_nearest * c
        y += dist_to_nearest * s

        # update dist_to_nearest for current point on ray
        # also keeps track of total ray length
        dist_to_nearest = distance_transform(x, y, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt)
        total_dist += dist_to_nearest
    
    return total_dist

@njit(cache=True)
def get_scan(pose, theta_dis, fov, num_beams, theta_index_increment, sines, cosines, eps, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt, max_range):
    """
    Perform the scan for each discretized angle of each beam of the laser, loop heavy, should be JITted

        Args:
            pose (numpy.ndarray(3, )): current pose of the scan frame in the map
            theta_dis (int): number of steps to discretize the angles between 0 and 2pi for look up
            fov (float): field of view of the laser scan
            num_beams (int): number of beams in the scan
            theta_index_increment (float): increment between angle indices after discretization

        Returns:
            scan (numpy.ndarray(n, )): resulting laser scan at the pose, n=num_beams
    """
    # empty scan array init
    scan = np.empty((num_beams,))

    # make theta discrete by mapping the range [-pi, pi] onto [0, theta_dis]
    theta_index = theta_dis * (pose[2] - fov/2.)/(2. * np.pi)

    # make sure it's wrapped properly
    theta_index = np.fmod(theta_index, theta_dis)
    while (theta_index < 0):
        theta_index += theta_dis

    # sweep through each beam
    for i in range(0, num_beams):
        # trace the current beam
        scan[i] = trace_ray(pose[0], pose[1], theta_index, sines, cosines, eps, orig_x, orig_y, orig_c, orig_s, height, width, resolution, dt, max_range)

        # increment the beam index
        theta_index += theta_index_increment

        # make sure it stays in the range [0, theta_dis)
        while theta_index >= theta_dis:
            theta_index -= theta_dis

    return scan

class ScanSimulator2D(object):
    """
    2D LIDAR scan simulator class

    Init params:
        num_beams (int): number of beams in the scan
        fov (float): field of view of the laser scan
        std_dev (float, default=0.01): standard deviation of the generated whitenoise in the scan
        eps (float, default=0.0001): ray tracing iteration termination condition
        theta_dis (int, default=2000): number of steps to discretize the angles between 0 and 2pi for look up
        max_range (float, default=30.0): maximum range of the laser
        seed (int, default=123): seed for random number generator for the whitenoise in scan
    """

    def __init__(self, num_beams, fov, std_dev=0.01, eps=0.0001, theta_dis=2000, max_range=30.0, seed=123):
        # initialization 
        self.num_beams = num_beams
        self.fov = fov
        self.std_dev = std_dev
        self.eps = eps
        self.theta_dis = theta_dis
        self.max_range = max_range
        self.angle_increment = self.fov / (self.num_beams - 1)
        self.theta_index_increment = theta_dis * self.angle_increment / (2. * np.pi)
        self.orig_c = None
        self.orig_s = None
        self.orig_x = None
        self.orig_y = None
        self.map_height = None
        self.map_width = None
        self.map_resolution = None
        self.dt = None
        
        # white noise generator
        self.rng = np.random.default_rng(seed=seed)

        # precomputing corresponding cosines and sines of the angle array
        theta_arr = np.linspace(0.0, 2*np.pi, num=theta_dis)
        self.sines = np.sin(theta_arr)
        self.cosines = np.cos(theta_arr)
    
    def set_map(self, map_path, map_ext):
        """
        Set the bitmap of the scan simulator by path

            Args:
                map_path (str): path to the map yaml file
                map_ext (str): extension (image type) of the map image

            Returns:
                flag (bool): if image reading and loading is successful
        """
        # TODO: do we open the option to flip the images, and turn rgb into grayscale? or specify the exact requirements in documentation.
        # TODO: throw error if image specification isn't met

        # load map image
        map_img_path = os.path.splitext(map_path)[0] + map_ext
        self.map_img = np.array(Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM))
        self.map_img = self.map_img.astype(np.float64)

        # grayscale -> binary
        self.map_img[self.map_img <= 128.] = 0.
        self.map_img[self.map_img > 128.] = 255.

        self.map_height = self.map_img.shape[0]
        self.map_width = self.map_img.shape[1]

        # load map yaml
        with open(map_path, 'r') as yaml_stream:
            try:
                map_metadata = yaml.safe_load(yaml_stream)
                self.map_resolution = map_metadata['resolution']
                self.origin = map_metadata['origin']
            except yaml.YAMLError as ex:
                print(ex)

        # calculate map parameters
        self.orig_x = self.origin[0]
        self.orig_y = self.origin[1]
        self.orig_s = np.sin(self.origin[2])
        self.orig_c = np.cos(self.origin[2])

        # get the distance transform
        self.dt = get_dt(self.map_img, self.map_resolution)

        return True

    def scan(self, pose):
        """
        Perform simulated 2D scan by pose on the given map

            Args:
                pose (numpy.ndarray (3, )): pose of the scan frame (x, y, theta)

            Returns:
                scan (numpy.ndarray (n, )): data array of the laserscan, n=num_beams

            Raises:
                ValueError: when scan is called before a map is set
        """
        if self.map_height is None:
            raise ValueError('Map is not set for scan simulator.')
        scan = get_scan(pose, self.theta_dis, self.fov, self.num_beams, self.theta_index_increment, self.sines, self.cosines, self.eps, self.orig_x, self.orig_y, self.orig_c, self.orig_s, self.map_height, self.map_width, self.map_resolution, self.dt, self.max_range)
        noise = self.rng.normal(0., self.std_dev, size=self.num_beams)
        final_scan = scan + noise
        return final_scan

    def get_increment(self):
        return self.angle_increment


"""
Unit tests for the 2D scan simulator class
Author: Hongrui Zheng

Test cases:
    1, 2: Comparison between generated scan array of the new simulator and the legacy C++ simulator, generated data used, MSE is used as the metric
    2. FPS test, should be greater than 500
"""


class ScanTests(unittest.TestCase):
    def setUp(self):
        # test params
        self.num_beams = 1080
        self.fov = 4.7
        
        self.num_test = 10
        self.test_poses = np.zeros((self.num_test, 3))
        self.test_poses[:, 2] = np.linspace(-1., 1., num=self.num_test)

        # legacy gym data
        sample_scan = np.load('legacy_scan.npz')
        self.berlin_scan = sample_scan['berlin']
        self.skirk_scan = sample_scan['skirk']

    def test_map_berlin(self):
        scan_sim = ScanSimulator2D(self.num_beams, self.fov)
        new_berlin = np.empty((self.num_test, self.num_beams))
        map_path = '../../../maps/berlin.yaml'
        map_ext = '.png'
        scan_sim.set_map(map_path, map_ext)
        # scan gen loop
        for i in range(self.num_test):
            test_pose = self.test_poses[i]
            new_berlin[i,:] = scan_sim.scan(test_pose)
        diff = self.berlin_scan - new_berlin
        mse = np.mean(diff**2)
        # print('Levine distance test, norm: ' + str(norm))

        # plotting
        import matplotlib.pyplot as plt
        theta = np.linspace(-self.fov/2., self.fov/2., num=self.num_beams)
        plt.polar(theta, new_berlin[1,:], '.', lw=0)
        plt.polar(theta, self.berlin_scan[1,:], '.', lw=0)
        plt.show()

        self.assertLess(mse, 2.)

    def test_map_skirk(self):
        scan_sim = ScanSimulator2D(self.num_beams, self.fov)
        new_skirk = np.empty((self.num_test, self.num_beams))
        map_path = '../../../maps/skirk.yaml'
        map_ext = '.png'
        scan_sim.set_map(map_path, map_ext)
        print('map set')
        # scan gen loop
        for i in range(self.num_test):
            test_pose = self.test_poses[i]
            new_skirk[i,:] = scan_sim.scan(test_pose)
        diff = self.skirk_scan - new_skirk
        mse = np.mean(diff**2)
        print('skirk distance test, mse: ' + str(mse))

        # plotting
        import matplotlib.pyplot as plt
        theta = np.linspace(-self.fov/2., self.fov/2., num=self.num_beams)
        plt.polar(theta, new_skirk[1,:], '.', lw=0)
        plt.polar(theta, self.skirk_scan[1,:], '.', lw=0)
        plt.show()

        self.assertLess(mse, 2.)

    def test_fps(self):
        # scan fps should be greater than 500
        scan_sim = ScanSimulator2D(self.num_beams, self.fov)
        map_path = '../../../maps/skirk.yaml'
        map_ext = '.png'
        scan_sim.set_map(map_path, map_ext)
        
        import time
        start = time.time()
        for i in range(10000):
            x_test = i/10000
            scan = scan_sim.scan(np.array([x_test, 0., 0.]))
        end = time.time()
        fps = 10000/(end-start)
        # print('FPS test')
        # print('Elapsed time: ' + str(end-start) + ' , FPS: ' + str(1/fps))
        self.assertGreater(fps, 500.)


def main():
    num_beams = 1080
    fov = 4.7
    # map_path = '../envs/maps/berlin.yaml'
    map_path = '/home/f1tenth-eval/tunercar/es/maps/map0.yaml'
    map_ext = '.png'
    scan_sim = ScanSimulator2D(num_beams, fov)
    scan_sim.set_map(map_path, map_ext)
    scan = scan_sim.scan(np.array([0., 0., 0.]))

    # fps test
    import time
    start = time.time()
    for i in range(10000):
        x_test = i/10000
        scan = scan_sim.scan(np.array([x_test, 0., 0.]))
    end = time.time()
    fps = (end-start)/10000
    print('FPS test')
    print('Elapsed time: ' + str(end-start) + ' , FPS: ' + str(1/fps))

    # visualization
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    num_iter = 100
    theta = np.linspace(-fov/2., fov/2., num=num_beams)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar')
    ax.set_ylim(0, 70)
    line, = ax.plot([], [], '.', lw=0)
    def update(i):
        # x_ani = i * 3. / num_iter
        theta_ani = -i * 2 * np.pi / num_iter
        x_ani = 0.
        current_scan = scan_sim.scan(np.array([x_ani, 0., theta_ani]))
        print(np.max(current_scan))
        line.set_data(theta, current_scan)
        return line, 
    ani = FuncAnimation(fig, update, frames=num_iter, blit=True)
    plt.show()

if __name__ == '__main__':
    # unittest.main()
    main()