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
Prototype of base classes
Replacement of the old RaceCar, Simulator classes in C++
Author: Hongrui Zheng
"""
from enum import Enum
import warnings

import numpy as np
from numba import njit

from f110_gym.envs.dynamic_models import vehicle_dynamics_st, pid
from f110_gym.envs.laser_models import ScanSimulator2D, check_ttc_jit, ray_cast
from f110_gym.envs.collision_models import get_vertices, collision_multiple

class Integrator(Enum):
    RK4 = 1
    Euler = 2


class RaceCar(object):
    """
    Base level race car class, handles the physics and laser scan of a single vehicle

    Data Members:
        params (dict): vehicle parameters dictionary
        is_ego (bool): ego identifier
        time_step (float): physics timestep
        num_beams (int): number of beams in laser
        fov (float): field of view of laser
        state (np.ndarray (7, )): state vector [x, y, theta, vel, steer_angle, ang_vel, slip_angle]
        odom (np.ndarray(13, )): odometry vector [x, y, z, qx, qy, qz, qw, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
        accel (float): current acceleration input
        steer_angle_vel (float): current steering velocity input
        in_collision (bool): collision indicator

    """

    # static objects that don't need to be stored in class instances
    scan_simulator = None
    cosines = None
    scan_angles = None
    side_distances = None

    def __init__(self, params, seed, is_ego=False, time_step=0.01, num_beams=1080, fov=4.7, integrator=Integrator.Euler):
        """
        Init function

        Args:
            params (dict): vehicle parameter dictionary, includes {'mu', 'C_Sf', 'C_Sr', 'lf', 'lr', 'h', 'm', 'I', 's_min', 's_max', 'sv_min', 'sv_max', 'v_switch', 'a_max': 9.51, 'v_min', 'v_max', 'length', 'width'}
            is_ego (bool, default=False): ego identifier
            time_step (float, default=0.01): physics sim time step
            num_beams (int, default=1080): number of beams in the laser scan
            fov (float, default=4.7): field of view of the laser

        Returns:
            None
        """

        # initialization
        self.params = params
        self.seed = seed
        self.is_ego = is_ego
        self.time_step = time_step
        self.num_beams = num_beams
        self.fov = fov
        self.integrator = integrator
        if self.integrator is Integrator.RK4:
            warnings.warn(f"Chosen integrator is RK4. This is different from previous versions of the gym.")

        # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]
        self.state = np.zeros((7, ))

        # pose of opponents in the world
        self.opp_poses = None

        # control inputs
        self.accel = 0.0
        self.steer_angle_vel = 0.0

        # steering delay buffer
        self.steer_buffer = np.empty((0, ))
        self.steer_buffer_size = 2

        # collision identifier
        self.in_collision = False

        # collision threshold for iTTC to environment
        self.ttc_thresh = 0.005

        # initialize scan sim
        if RaceCar.scan_simulator is None:
            self.scan_rng = np.random.default_rng(seed=self.seed)
            RaceCar.scan_simulator = ScanSimulator2D(num_beams, fov)

            scan_ang_incr = RaceCar.scan_simulator.get_increment()

            # angles of each scan beam, distance from lidar to edge of car at each beam, and precomputed cosines of each angle
            RaceCar.cosines = np.zeros((num_beams, ))
            RaceCar.scan_angles = np.zeros((num_beams, ))
            RaceCar.side_distances = np.zeros((num_beams, ))

            dist_sides = params['width']/2.
            dist_fr = (params['lf']+params['lr'])/2.

            for i in range(num_beams):
                angle = -fov/2. + i*scan_ang_incr
                RaceCar.scan_angles[i] = angle
                RaceCar.cosines[i] = np.cos(angle)

                if angle > 0:
                    if angle < np.pi/2:
                        # between 0 and pi/2
                        to_side = dist_sides / np.sin(angle)
                        to_fr = dist_fr / np.cos(angle)
                        RaceCar.side_distances[i] = min(to_side, to_fr)
                    else:
                        # between pi/2 and pi
                        to_side = dist_sides / np.cos(angle - np.pi/2.)
                        to_fr = dist_fr / np.sin(angle - np.pi/2.)
                        RaceCar.side_distances[i] = min(to_side, to_fr)
                else:
                    if angle > -np.pi/2:
                        # between 0 and -pi/2
                        to_side = dist_sides / np.sin(-angle)
                        to_fr = dist_fr / np.cos(-angle)
                        RaceCar.side_distances[i] = min(to_side, to_fr)
                    else:
                        # between -pi/2 and -pi
                        to_side = dist_sides / np.cos(-angle - np.pi/2)
                        to_fr = dist_fr / np.sin(-angle - np.pi/2)
                        RaceCar.side_distances[i] = min(to_side, to_fr)

    def update_params(self, params):
        """
        Updates the physical parameters of the vehicle
        Note that does not need to be called at initialization of class anymore

        Args:
            params (dict): new parameters for the vehicle

        Returns:
            None
        """
        self.params = params
    
    def set_map(self, map_path, map_ext):
        """
        Sets the map for scan simulator
        
        Args:
            map_path (str): absolute path to the map yaml file
            map_ext (str): extension of the map image file
        """
        RaceCar.scan_simulator.set_map(map_path, map_ext)

    def reset(self, pose):
        """
        Resets the vehicle to a pose
        
        Args:
            pose (np.ndarray (3, )): pose to reset the vehicle to

        Returns:
            None
        """
        # clear control inputs
        self.accel = 0.0
        self.steer_angle_vel = 0.0
        # clear collision indicator
        self.in_collision = False
        # clear state
        self.state = np.zeros((7, ))
        self.state[0:2] = pose[0:2]
        self.state[4] = pose[2]
        self.steer_buffer = np.empty((0, ))
        # reset scan random generator
        self.scan_rng = np.random.default_rng(seed=self.seed)

    def ray_cast_agents(self, scan):
        """
        Ray cast onto other agents in the env, modify original scan

        Args:
            scan (np.ndarray, (n, )): original scan range array

        Returns:
            new_scan (np.ndarray, (n, )): modified scan
        """

        # starting from original scan
        new_scan = scan
        
        # loop over all opponent vehicle poses
        for opp_pose in self.opp_poses:
            # get vertices of current oppoenent
            opp_vertices = get_vertices(opp_pose, self.params['length'], self.params['width'])

            new_scan = ray_cast(np.append(self.state[0:2], self.state[4]), new_scan, self.scan_angles, opp_vertices)

        return new_scan

    def check_ttc(self, current_scan):
        """
        Check iTTC against the environment, sets vehicle states accordingly if collision occurs.
        Note that this does NOT check collision with other agents.

        state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]

        Args:
            current_scan

        Returns:
            None
        """
        
        in_collision = check_ttc_jit(current_scan, self.state[3], self.scan_angles, self.cosines, self.side_distances, self.ttc_thresh)

        # if in collision stop vehicle
        if in_collision:
            self.state[3:] = 0.
            self.accel = 0.0
            self.steer_angle_vel = 0.0

        # update state
        self.in_collision = in_collision

        return in_collision

    def update_pose(self, raw_steer, vel):
        """
        Steps the vehicle's physical simulation

        Args:
            steer (float): desired steering angle
            vel (float): desired longitudinal velocity

        Returns:
            current_scan
        """

        # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]

        # steering delay
        steer = 0.
        if self.steer_buffer.shape[0] < self.steer_buffer_size:
            steer = 0.
            self.steer_buffer = np.append(raw_steer, self.steer_buffer)
        else:
            steer = self.steer_buffer[-1]
            self.steer_buffer = self.steer_buffer[:-1]
            self.steer_buffer = np.append(raw_steer, self.steer_buffer)


        # steering angle velocity input to steering velocity acceleration input
        accl, sv = pid(vel, steer, self.state[3], self.state[2], self.params['sv_max'], self.params['a_max'], self.params['v_max'], self.params['v_min'])
        
        if self.integrator is Integrator.RK4:
            # RK4 integration
            k1 = vehicle_dynamics_st(
                self.state,
                np.array([sv, accl]),
                self.params['mu'],
                self.params['C_Sf'],
                self.params['C_Sr'],
                self.params['lf'],
                self.params['lr'],
                self.params['h'],
                self.params['m'],
                self.params['I'],
                self.params['s_min'],
                self.params['s_max'],
                self.params['sv_min'],
                self.params['sv_max'],
                self.params['v_switch'],
                self.params['a_max'],
                self.params['v_min'],
                self.params['v_max'])

            k2_state = self.state + self.time_step*(k1/2)

            k2 = vehicle_dynamics_st(
                k2_state,
                np.array([sv, accl]),
                self.params['mu'],
                self.params['C_Sf'],
                self.params['C_Sr'],
                self.params['lf'],
                self.params['lr'],
                self.params['h'],
                self.params['m'],
                self.params['I'],
                self.params['s_min'],
                self.params['s_max'],
                self.params['sv_min'],
                self.params['sv_max'],
                self.params['v_switch'],
                self.params['a_max'],
                self.params['v_min'],
                self.params['v_max'])

            k3_state = self.state + self.time_step*(k2/2)

            k3 = vehicle_dynamics_st(
                k3_state,
                np.array([sv, accl]),
                self.params['mu'],
                self.params['C_Sf'],
                self.params['C_Sr'],
                self.params['lf'],
                self.params['lr'],
                self.params['h'],
                self.params['m'],
                self.params['I'],
                self.params['s_min'],
                self.params['s_max'],
                self.params['sv_min'],
                self.params['sv_max'],
                self.params['v_switch'],
                self.params['a_max'],
                self.params['v_min'],
                self.params['v_max'])

            k4_state = self.state + self.time_step*k3

            k4 = vehicle_dynamics_st(
                k4_state,
                np.array([sv, accl]),
                self.params['mu'],
                self.params['C_Sf'],
                self.params['C_Sr'],
                self.params['lf'],
                self.params['lr'],
                self.params['h'],
                self.params['m'],
                self.params['I'],
                self.params['s_min'],
                self.params['s_max'],
                self.params['sv_min'],
                self.params['sv_max'],
                self.params['v_switch'],
                self.params['a_max'],
                self.params['v_min'],
                self.params['v_max'])

            # dynamics integration
            self.state = self.state + self.time_step*(1/6)*(k1 + 2*k2 + 2*k3 + k4)
        
        elif self.integrator is Integrator.Euler:
            f = vehicle_dynamics_st(
                self.state,
                np.array([sv, accl]),
                self.params['mu'],
                self.params['C_Sf'],
                self.params['C_Sr'],
                self.params['lf'],
                self.params['lr'],
                self.params['h'],
                self.params['m'],
                self.params['I'],
                self.params['s_min'],
                self.params['s_max'],
                self.params['sv_min'],
                self.params['sv_max'],
                self.params['v_switch'],
                self.params['a_max'],
                self.params['v_min'],
                self.params['v_max'])
            self.state = self.state + self.time_step * f
        
        else:
            raise SyntaxError(f"Invalid Integrator Specified. Provided {self.integrator.name}. Please choose RK4 or Euler")

        # bound yaw angle
        if self.state[4] > 2*np.pi:
            self.state[4] = self.state[4] - 2*np.pi
        elif self.state[4] < 0:
            self.state[4] = self.state[4] + 2*np.pi

        # update scan
        current_scan = RaceCar.scan_simulator.scan(np.append(self.state[0:2], self.state[4]), self.scan_rng)

        return current_scan

    def update_opp_poses(self, opp_poses):
        """
        Updates the vehicle's information on other vehicles

        Args:
            opp_poses (np.ndarray(num_other_agents, 3)): updated poses of other agents

        Returns:
            None
        """
        self.opp_poses = opp_poses


    def update_scan(self, agent_scans, agent_index):
        """
        Steps the vehicle's laser scan simulation
        Separated from update_pose because needs to update scan based on NEW poses of agents in the environment

        Args:
            agent scans list (modified in-place),
            agent index (int)

        Returns:
            None
        """

        current_scan = agent_scans[agent_index]

        # check ttc
        self.check_ttc(current_scan)

        # ray cast other agents to modify scan
        new_scan = self.ray_cast_agents(current_scan)

        agent_scans[agent_index] = new_scan

class Simulator(object):
    """
    Simulator class, handles the interaction and update of all vehicles in the environment

    Data Members:
        num_agents (int): number of agents in the environment
        time_step (float): physics time step
        agent_poses (np.ndarray(num_agents, 3)): all poses of all agents
        agents (list[RaceCar]): container for RaceCar objects
        collisions (np.ndarray(num_agents, )): array of collision indicator for each agent
        collision_idx (np.ndarray(num_agents, )): which agent is each agent in collision with

    """

    def __init__(self, params, num_agents, seed, time_step=0.01, ego_idx=0, integrator=Integrator.RK4):
        """
        Init function

        Args:
            params (dict): vehicle parameter dictionary, includes {'mu', 'C_Sf', 'C_Sr', 'lf', 'lr', 'h', 'm', 'I', 's_min', 's_max', 'sv_min', 'sv_max', 'v_switch', 'a_max', 'v_min', 'v_max', 'length', 'width'}
            num_agents (int): number of agents in the environment
            seed (int): seed of the rng in scan simulation
            time_step (float, default=0.01): physics time step
            ego_idx (int, default=0): ego vehicle's index in list of agents

        Returns:
            None
        """
        self.num_agents = num_agents
        self.seed = seed
        self.time_step = time_step
        self.ego_idx = ego_idx
        self.params = params
        self.agent_poses = np.empty((self.num_agents, 3))
        self.agents = []
        self.collisions = np.zeros((self.num_agents, ))
        self.collision_idx = -1 * np.ones((self.num_agents, ))

        # initializing agents
        for i in range(self.num_agents):
            if i == ego_idx:
                ego_car = RaceCar(params, self.seed, is_ego=True, time_step=self.time_step, integrator=integrator)
                self.agents.append(ego_car)
            else:
                agent = RaceCar(params, self.seed, is_ego=False, time_step=self.time_step, integrator=integrator)
                self.agents.append(agent)

    def set_map(self, map_path, map_ext):
        """
        Sets the map of the environment and sets the map for scan simulator of each agent

        Args:
            map_path (str): path to the map yaml file
            map_ext (str): extension for the map image file

        Returns:
            None
        """
        for agent in self.agents:
            agent.set_map(map_path, map_ext)


    def update_params(self, params, agent_idx=-1):
        """
        Updates the params of agents, if an index of an agent is given, update only that agent's params

        Args:
            params (dict): dictionary of params, see details in docstring of __init__
            agent_idx (int, default=-1): index for agent that needs param update, if negative, update all agents

        Returns:
            None
        """
        if agent_idx < 0:
            # update params for all
            for agent in self.agents:
                agent.update_params(params)
        elif agent_idx >= 0 and agent_idx < self.num_agents:
            # only update one agent's params
            self.agents[agent_idx].update_params(params)
        else:
            # index out of bounds, throw error
            raise IndexError('Index given is out of bounds for list of agents.')

    def check_collision(self):
        """
        Checks for collision between agents using GJK and agents' body vertices

        Args:
            None

        Returns:
            None
        """
        # get vertices of all agents
        all_vertices = np.empty((self.num_agents, 4, 2))
        for i in range(self.num_agents):
            all_vertices[i, :, :] = get_vertices(np.append(self.agents[i].state[0:2],self.agents[i].state[4]), self.params['length'], self.params['width'])
        self.collisions, self.collision_idx = collision_multiple(all_vertices)


    def step(self, control_inputs):
        """
        Steps the simulation environment

        Args:
            control_inputs (np.ndarray (num_agents, 2)): control inputs of all agents, first column is desired steering angle, second column is desired velocity
        
        Returns:
            observations (dict): dictionary for observations: poses of agents, current laser scan of each agent, collision indicators, etc.
        """


        agent_scans = []

        # looping over agents
        for i, agent in enumerate(self.agents):
            # update each agent's pose
            current_scan = agent.update_pose(control_inputs[i, 0], control_inputs[i, 1])
            agent_scans.append(current_scan)

            # update sim's information of agent poses
            self.agent_poses[i, :] = np.append(agent.state[0:2], agent.state[4])

        # check collisions between all agents
        self.check_collision()

        for i, agent in enumerate(self.agents):
            # update agent's information on other agents
            opp_poses = np.concatenate((self.agent_poses[0:i, :], self.agent_poses[i+1:, :]), axis=0)
            agent.update_opp_poses(opp_poses)

            # update each agent's current scan based on other agents
            agent.update_scan(agent_scans, i)

            # update agent collision with environment
            if agent.in_collision:
                self.collisions[i] = 1.

        # fill in observations
        # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]
        # collision_angles is removed from observations
        observations = {'ego_idx': self.ego_idx,
            'scans': [],
            'poses_x': [],
            'poses_y': [],
            'poses_theta': [],
            'linear_vels_x': [],
            'linear_vels_y': [],
            'ang_vels_z': [],
            'collisions': self.collisions}
        for i, agent in enumerate(self.agents):
            observations['scans'].append(agent_scans[i])
            observations['poses_x'].append(agent.state[0])
            observations['poses_y'].append(agent.state[1])
            observations['poses_theta'].append(agent.state[4])
            observations['linear_vels_x'].append(agent.state[3])
            observations['linear_vels_y'].append(0.)
            observations['ang_vels_z'].append(agent.state[5])

        return observations

    def reset(self, poses):
        """
        Resets the simulation environment by given poses

        Arges:
            poses (np.ndarray (num_agents, 3)): poses to reset agents to

        Returns:
            None
        """
        
        if poses.shape[0] != self.num_agents:
            raise ValueError('Number of poses for reset does not match number of agents.')

        # loop over poses to reset
        for i in range(self.num_agents):
            self.agents[i].reset(poses[i, :])
