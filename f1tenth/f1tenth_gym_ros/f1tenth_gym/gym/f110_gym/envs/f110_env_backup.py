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

'''
Author: Hongrui Zheng
'''

# gym imports
import gym
from gym import error, spaces, utils
from gym.utils import seeding

# zmq imports
import zmq

# protobuf import
import sim_requests_pb2

# others
import numpy as np

from numba import njit
from scipy.ndimage import distance_transform_edt as edt

from PIL import Image
import sys
import os
import signal
import subprocess
import math
import yaml
import csv

# from matplotlib.pyplot import imshow
# import matplotlib.pyplot as plt

class F110Env(gym.Env, utils.EzPickle):
    """
    OpenAI gym environment for F1/10 simulator
    Use 0mq's REQ-REP pattern to communicate to the C++ simulator
    ONE env has ONE corresponding C++ instance
    Need to create env with map input, full path to map yaml file, map pgm image and yaml should be in same directory

    should be initialized with a map, a timestep, and number of agents
    """
    metadata = {'render.modes': []}

    def __init__(self):
        # simualtor params
        self.params_set = False
        self.map_inited = False
        # params list is [mu, h_cg, l_r, cs_f, cs_r, I_z, mass]
        self.params = []
        # TODO: add multi agent stuff, need a _add_agent function of sth
        self.num_agents = 2
        self.timestep = 0.01

        # TODO: clean up the map path stuff, right now it's a init_map function
        self.map_path = None
        self.map_img = None

        # current_dir = os.path.dirname(os.path.abspath(__file__))
        # map_path = current_dir + '/../../../maps/levine.yaml'

        # default
        self.ego_idx = 0

        # TODO: also set these things in init function?
        self.timeout = 120.0
        # radius to consider done
        self.start_thresh = 0.5  # 10cm

        # env states
        # more accurate description should be ego car state
        # might not need to keep scan
        self.x = None
        self.y = None
        self.theta = None

        self.in_collision = False
        self.collision_angle = None

        # loop completion
        self.near_start = True
        self.num_toggles = 0

        # race info
        self.lap_times = [0.0, 0.0]
        self.lap_counts = [0, 0]

        # TODO: load the map (same as ROS .yaml format)
        # if not map_path.endswith('.yaml'):
        #     print('Gym env - Please use a yaml file for map input.')
        #     sys.exit()
        # load map img
        # map_img_path = 'levine.png'
        # self.map_img = cv2.imread(map_img_path, 0)
        # self.map_img = cv2.flip(self.map_img, 0)
        # self.map_img = np.array(Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM))
        # self.map_img = self.map_img.astype(np.float64)
        # self.map_img = self.map_img[::-1]
        # self.map_img = np.dot(self.map_img[..., :3], [0.29, 0.57, 0.14])
        # plt.imshow(self.map_img)
        # plt.show()

        # map metadata
        # self.map_height = self.map_img.shape[0]
        # self.map_width = self.map_img.shape[1]
        self.map_height = 0.0
        self.map_width = 0.0
        self.map_resolution = 0.0
        self.free_thresh = 0.0
        self.origin = []
        # load map metadata
        # with open(map_path, 'r') as yaml_stream:
        #     try:
        #         map_metadata = yaml.safe_load(yaml_stream)
        #         self.map_resolution = map_metadata['resolution']
        #         self.origin = map_metadata['origin']
        #         # print(self.origin)
        #         # self.free_thresh?????
        #     except yaml.YAMLError as ex:
        #         print(ex)

        # create zmq stuff
        # port number range from 6666 - 6766
        # max 100 tries to connect/bind
        tries = 0
        max_tries = 100
        min_port = 6666
        self.port = min_port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        while tries < max_tries:
            try:
                self.socket.bind('tcp://*:%s' % str(min_port + tries))
                # self.socket.connect('tcp://localhost:6666')
                self.port = min_port + tries
                break
            except:
                tries = tries + 1
                # print('Gym env - retrying for ' + str(tries) + ' times')

        print('Gym env - Connected env to port: ' + str(self.port))

        # create cpp instance if create then need to pass port number
        # subprocess call assumes directory structure
        # init sim with arguments: [ex timestep num_agents port_num]
        # TODO: include other car params in argument
        # args = ['../build/sim_server', str(self.timestep), str(self.num_agents), str(self.port)]
        # self.sim_p = subprocess.Popen(args)
        self.sim_p = None

        # print('Gym env - env created, waiting for params...')

    def __del__(self):
        """
        Finalizer, does cleanup
        """
        if self.sim_p is None:
            pass
        else:
            os.kill(self.sim_p.pid, signal.SIGTERM)
            # print('Gym env - Sim child process killed.')

    def _start_executable(self, path):
        mu = self.params[0]
        h_cg = self.params[1]
        l_r = self.params[2]
        cs_f = self.params[3]
        cs_r = self.params[4]
        I_z = self.params[5]
        mass = self.params[6]
        args = [path+'sim_server', str(self.timestep), str(self.num_agents), str(self.port), str(mu), str(h_cg), str(l_r), str(cs_f), str(cs_r), str(I_z), str(mass)]
        self.sim_p = subprocess.Popen(args)

    def _set_map(self):
        """
        Sets the map for the simulator instance
        """
        if not self.map_inited:
            print('Gym env - Sim map not initialized, call env.init_map() to init map.')
        # create and fill in protobuf
        map_request_proto = sim_requests_pb2.SimRequest()
        map_request_proto.type = 1
        map_request_proto.map_request.map.extend((1. - self.map_img/255.).flatten().tolist())
        map_request_proto.map_request.origin_x = self.origin[0]
        map_request_proto.map_request.origin_y = self.origin[1]
        map_request_proto.map_request.map_resolution = self.map_resolution
        # TODO: double check if this value is valid
        map_request_proto.map_request.free_threshold = self.free_thresh
        map_request_proto.map_request.map_height = self.map_height
        map_request_proto.map_request.map_width = self.map_width
        # serialization
        map_request_string = map_request_proto.SerializeToString()
        # send set map request
        # print('Gym env - Sending set map request...')
        self.socket.send(map_request_string)
        # print('Gym env - Map request sent.')
        # receive response from sim instance
        sim_response_string = self.socket.recv()
        # parse map response proto
        sim_response_proto = sim_requests_pb2.SimResponse()
        sim_response_proto.ParseFromString(sim_response_string)
        # get results
        set_map_result = sim_response_proto.map_result.result
        if set_map_result == 1:
            print('Gym env - Set map failed, exiting...')
            sys.exit()

    def _check_done(self):
        """
        Check if the episode is done
        This is in terms of the ego car
        For our case, whether the car ends up close enough to the starting point
        And if accumulated time is over the timeout
        return true if done, false if not
        This assumes start is always (0, 0)

        """
        # TODO: start not always 0, 0
        # dist_to_start = math.sqrt((self.x-self.start_x) ** 2 + (self.y-self.start_y) ** 2)
        left_t = 2
        right_t = 2
        timeout = self.current_time >= self.timeout
        if self.double_finish:
            poses_x = np.array(self.all_x)-self.start_xs
            poses_y = np.array(self.all_y)-self.start_ys
            delta_pt = np.dot(self.start_rot, np.stack((poses_x, poses_y), axis=0))
            temp_y = delta_pt[1,:]
            idx1 = temp_y > left_t
            idx2 = temp_y < -right_t
            temp_y[idx1] -= left_t
            temp_y[idx2] = -right_t - temp_y[idx2]
            temp_y[np.invert(np.logical_or(idx1, idx2))] = 0

            dist2 = delta_pt[0,:]**2 + temp_y**2
            closes = dist2 <= 0.1
            for i in range(self.num_agents):
                if closes[i] and not self.near_starts[i]:
                    self.near_starts[i] = True
                    self.toggle_list[i] += 1
                elif not closes[i] and self.near_starts[i]:
                    self.near_starts[i] = False
                    self.toggle_list[i] += 1
            done = (self.in_collision | (timeout) | np.all(self.toggle_list >= 4))
            # only for two cars atm
            self.lap_counts[0] = np.floor(self.toggle_list[0] / 2)
            self.lap_counts[1] = np.floor(self.toggle_list[1] / 2)
            if self.toggle_list[0] < 4:
                self.lap_times[0] = self.current_time
            if self.toggle_list[1] < 4:
                self.lap_times[1] = self.current_time
            return done, self.toggle_list >= 4

        delta_pt = np.dot(self.start_rot, np.array([self.x-self.start_x, self.y-self.start_y]))
        if delta_pt[1] > left_t: # left
            temp_y = delta_pt[1]-left_t
        elif delta_pt[1] < -right_t: # right
            temp_y = -right_t - delta_pt[1]
        else:
            temp_y = 0
        dist2 = delta_pt[0]**2 + temp_y**2
        close = dist2 <= 0.1
        # close = dist_to_start <= self.start_thresh
        if close and not self.near_start:
            self.near_start = True
            self.num_toggles += 1
        elif not close and self.near_start:
            self.near_start = False
            self.num_toggles += 1
        done = (self.in_collision | (timeout) | (self.num_toggles >= 4))
        return done

    def _check_passed(self):
        """
        Returns the times that the ego car overtook the other car
        """
        return 0

    def _update_state(self, obs_dict):
        """
        Update the env's states according to observations
        obs is observation dictionary
        """
        self.x = obs_dict['poses_x'][obs_dict['ego_idx']]
        self.y = obs_dict['poses_y'][obs_dict['ego_idx']]
        if self.double_finish:
            self.all_x = obs_dict['poses_x']
            self.all_y = obs_dict['poses_y']

        self.theta = obs_dict['poses_theta'][obs_dict['ego_idx']]
        self.in_collision = obs_dict['collisions'][obs_dict['ego_idx']]
        self.collision_angle = obs_dict['collision_angles'][obs_dict['ego_idx']]

    # TODO: do we do the ray casting here or in C++?
    # if speed is a concern do it in C++?
    # numba shouldn't be a dependency of gym env
    def _raycast_opponents(self, obs_dict):
        # find the angle of beam of each car in each other's fov

        # set range of beams to raycast, ego and op

        # raycast beams, two set
        new_obs = {}
        return new_obs

    def step(self, action):
        # can't step if params not set
        if not self.params_set:
            print('ERROR - Gym Env - Params not set, call update params before stepping.')
            sys.exit()
        # action is a list of steering angles + command velocities
        # also a ego car index
        # action should a DICT with {'ego_idx': int, 'speed':[], 'steer':[]}
        step_request_proto = sim_requests_pb2.SimRequest()
        step_request_proto.type = 0
        step_request_proto.step_request.ego_idx = action['ego_idx']
        step_request_proto.step_request.requested_vel.extend(action['speed'])
        step_request_proto.step_request.requested_ang.extend(action['steer'])
        # serialization
        step_request_string = step_request_proto.SerializeToString()
        # send step request
        self.socket.send(step_request_string)
        # receive response from sim instance
        sim_response_string = self.socket.recv()
        # print('Gym env - Received response for step request.')
        # parse map response proto
        sim_response_proto = sim_requests_pb2.SimResponse()
        sim_response_proto.ParseFromString(sim_response_string)
        # get results
        # make sure we have the right type of response
        response_type = sim_response_proto.type
        # TODO: also check for stepping fail
        if not response_type == 0:
            print('Gym env - Wrong response type for stepping, exiting...')
            sys.exit()
        observations_proto = sim_response_proto.sim_obs
        # make sure the ego idx matches
        if not observations_proto.ego_idx == action['ego_idx']:
            print('Gym env - Ego index mismatch, exiting...')
            sys.exit()
        # get observations
        carobs_list = observations_proto.observations
        # construct observation dict
        # Observation DICT, assume indices consistent: {'ego_idx':int, 'scans':[[]], 'poses_x':[], 'poses_y':[], 'poses_theta':[], 'linear_vels_x':[], 'linear_vels_y':[], 'ang_vels_z':[], 'collisions':[], 'collision_angles':[]}
        obs = {'ego_idx': observations_proto.ego_idx, 'scans': [], 'poses_x': [], 'poses_y': [], 'poses_theta': [], 'linear_vels_x': [], 'linear_vels_y': [], 'ang_vels_z': [], 'collisions': [], 'collision_angles': [], 'lap_times': [], 'lap_counts': []}
        for car_obs in carobs_list:
            obs['scans'].append(car_obs.scan)
            obs['poses_x'].append(car_obs.pose_x)
            obs['poses_y'].append(car_obs.pose_y)
            if abs(car_obs.theta) < np.pi:
                obs['poses_theta'].append(car_obs.theta)
            else:
                obs['poses_theta'].append(-((2 * np.pi) - car_obs.theta))
            obs['linear_vels_x'].append(car_obs.linear_vel_x)
            obs['linear_vels_y'].append(car_obs.linear_vel_y)
            obs['ang_vels_z'].append(car_obs.ang_vel_z)
            obs['collisions'].append(car_obs.collision)
            obs['collision_angles'].append(car_obs.collision_angle)

        obs['lap_times'] = self.lap_times
        obs['lap_counts'] = self.lap_counts

        # TODO: do we need step reward?
        reward = self.timestep
        # update accumulated time in env
        self.current_time = self.current_time + self.timestep
        # TODO: donezo should be done in simulator? could be done here as well
        self._update_state(obs)
        if self.double_finish:
            done, temp = self._check_done()
            info = {'checkpoint_done': temp}
        else:
            done = self._check_done()
            info = {}

        # TODO: return obs, reward, done, info
        return obs, reward, done, info

    def reset(self, poses=None):

        self.current_time = 0.0
        self.in_collision = False
        self.collision_angles = None
        self.num_toggles = 0
        self.near_start = True
        self.near_starts = np.array([True]*self.num_agents)
        self.toggle_list = np.zeros((self.num_agents,))
        if poses:
            pose_x = poses['x']
            pose_y = poses['y']
            pose_theta = poses['theta']
            self.start_x = pose_x[0]
            self.start_y = pose_y[0]
            self.start_theta = pose_theta[0]
            self.start_xs = np.array(pose_x)
            self.start_ys = np.array(pose_y)
            self.start_thetas = np.array(pose_theta)
            self.start_rot = np.array([[np.cos(-self.start_theta), -np.sin(-self.start_theta)],
                                        [np.sin(-self.start_theta), np.cos(-self.start_theta)]])
            # create reset by pose proto
            reset_request_proto = sim_requests_pb2.SimRequest()
            reset_request_proto.type = 4
            reset_request_proto.reset_bypose_request.num_cars = self.num_agents
            reset_request_proto.reset_bypose_request.ego_idx = 0
            reset_request_proto.reset_bypose_request.car_x.extend(pose_x)
            reset_request_proto.reset_bypose_request.car_y.extend(pose_y)
            reset_request_proto.reset_bypose_request.car_theta.extend(pose_theta)
            reset_request_string = reset_request_proto.SerializeToString()
            self.socket.send(reset_request_string)
        else:
            # create reset proto
            self.start_x = 0.0
            self.start_y = 0.0
            self.start_theta = 0.0
            self.start_rot = np.array([[np.cos(-self.start_theta), -np.sin(-self.start_theta)],
                                        [np.sin(-self.start_theta), np.cos(-self.start_theta)]])
            reset_request_proto = sim_requests_pb2.SimRequest()
            reset_request_proto.type = 2
            reset_request_proto.reset_request.num_cars = self.num_agents
            reset_request_proto.reset_request.ego_idx = 0
            # serialize reset proto
            reset_request_string = reset_request_proto.SerializeToString()
            # send reset proto string
            self.socket.send(reset_request_string)
        # receive response from sim
        reset_response_string = self.socket.recv()
        reset_response_proto = sim_requests_pb2.SimResponse()
        reset_response_proto.ParseFromString(reset_response_string)
        if reset_response_proto.reset_resp.result:
            print('Gym env - Reset failed')
            # TODO: failure handling
            return None
        # TODO: return with gym convention, one step?
        vels = [0.0] * self.num_agents
        angs = [0.0] * self.num_agents
        action = {'ego_idx': self.ego_idx, 'speed': vels, 'steer': angs}
        # print('Gym env - Reset done')
        obs, reward, done, info = self.step(action)
        # print('Gym env - step done for reset')
        return obs, reward, done, info

    def init_map(self, map_path, img_ext, rgb, flip):
        """
            init a map for the gym env
            map_path: full path for the yaml, same as ROS, img and yaml in same dir
            rgb: map grayscale or rgb
            flip: if map needs flipping
        """

        self.map_path = map_path
        if not map_path.endswith('.yaml'):
            print('Gym env - Please use a yaml file for map initialization.')
            print('Exiting...')
            sys.exit()

        # split yaml ext name
        map_img_path = os.path.splitext(self.map_path)[0] + img_ext
        self.map_img = np.array(Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM))
        self.map_img = self.map_img.astype(np.float64)
        if flip:
            self.map_img = self.map_img[::-1]

        if rgb:
            self.map_img = np.dot(self.map_img[..., :3], [0.29, 0.57, 0.14])

        # update map metadata
        self.map_height = self.map_img.shape[0]
        self.map_width = self.map_img.shape[1]
        self.free_thresh = 0.6  # TODO: double check
        with open(self.map_path, 'r') as yaml_stream:
            try:
                map_metadata = yaml.safe_load(yaml_stream)
                self.map_resolution = map_metadata['resolution']
                self.origin = map_metadata['origin']
            except yaml.YAMLError as ex:
                print(ex)
        self.map_inited = True

        # load waypoints
        # self.csv_path = os.path.splitext(self.map_path)[0] + '.csv'
        # with open(self.csv_path) as f:
        #     self.waypoints = [tuple(line) for line in csv.reader(f)]
        #     # waypoints are [x, y, speed, theta]
        #     self.waypoints = np.array([(float(pt[0]), float(pt[1]), float(pt[2]), float(pt[3])) for pt in self.waypoints])


    def render(self, mode='human', close=False):
        return

    # def get_min_dist(self, position):
    #     wpts = self.waypoints[:, 0:2]
    #      # = position[0:2]
    #     nearest_point, nearest_dist, t, i = self.nearest_point_on_trajectory(position, wpts)
    #     # speed = self.waypoints[i, 2]
    #     return nearest_dist

    # def nearest_point_on_trajectory(self, point, trajectory):
    #     '''
    #     Return the nearest point along the given piecewise linear trajectory.

    #     Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
    #     not be an issue so long as trajectories are not insanely long.

    #         Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)

    #     point: size 2 numpy array
    #     trajectory: Nx2 matrix of (x,y) trajectory waypoints
    #         - these must be unique. If they are not unique, a divide by 0 error will destroy the world
    #     '''
    #     diffs = trajectory[1:,:] - trajectory[:-1,:]
    #     l2s   = diffs[:,0]**2 + diffs[:,1]**2
    #     # this is equivalent to the elementwise dot product
    #     dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
    #     t = np.clip(dots / l2s, 0.0, 1.0)
    #     projections = trajectory[:-1,:] + (t*diffs.T).T
    #     dists = np.linalg.norm(point - projections,axis=1)
    #     min_dist_segment = np.argmin(dists)
    #     return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment

    def update_params(self, mu, h_cg, l_r, cs_f, cs_r, I_z, mass, exe_path, double_finish=False):
        # if not self.sim_p is None:
        #     print('Gym env - Sim server exists, killing...')
        #     self.socket.send(b'dead')
        #     self.sim_p.kill()
        #     os.kill(self.sim_p.pid, signal.SIGINT)
        #     self.sim_p = None
        # print('in update params')

        self.params = [mu, h_cg, l_r, cs_f, cs_r, I_z, mass]
        self.params_set = True
        if self.sim_p is None:
            # print('starting ex and setting map')
            self._start_executable(exe_path)
            self._set_map()
        self.double_finish = double_finish
        # print('before creating proto')

        # create update proto
        update_param_proto = sim_requests_pb2.SimRequest()
        update_param_proto.type = 3
        update_param_proto.update_request.mu = mu
        update_param_proto.update_request.h_cg = h_cg
        update_param_proto.update_request.l_r = l_r
        update_param_proto.update_request.cs_f = cs_f
        update_param_proto.update_request.cs_r = cs_r
        update_param_proto.update_request.I_z = I_z
        update_param_proto.update_request.mass = mass
        # serialize reset proto
        update_param_string = update_param_proto.SerializeToString()
        # print('proto serialized')
        # send update param request
        self.socket.send(update_param_string)
        # print('Gym env - Update param request sent.')
        # receive response
        update_response_string = self.socket.recv()
        update_response_proto = sim_requests_pb2.SimResponse()
        update_response_proto.ParseFromString(update_response_string)
        if update_response_proto.update_resp.result:
            print('Gym env - Update param failed')
            return None

        # print('Gym env - params updated.')
        # start executable
        # self._start_executable()
        # call set map
        # self._set_map()
