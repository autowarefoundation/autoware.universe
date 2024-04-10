import freespace_planning_algorithms.astar_search as fp

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.serialization import serialize_message

from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint

import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd
from pyquaternion import Quaternion
import time
import pickle
from tqdm import tqdm

import sys
import os
sys.path.append(os.path.dirname(__file__))
from param.astar_params import vehicle_shape, planner_param, astar_param
from common.common_classes import resultBag


def createTrajectory(waypoints):
    trajectory = Trajectory()
    for waypoint in waypoints.waypoints:
        trajectory_point = TrajectoryPoint()
        trajectory_point.pose = waypoint
        trajectory.points.append(trajectory_point)

    return trajectory

class AstarPythonNode(Node):
    def __init__(self, msg):
        # Node.__init__を引数node_nameにtalkerを渡して継承
        super().__init__('astar_python')
        self.count = 0
        self.msg = msg

        # Node.create_publisher(msg_type, topic)に引数を渡してpublisherを作成
        self.pub_trajectory = self.create_publisher(Trajectory, 'planning/freespace_param_tuning/trajectory', 1)
        # self.sub_point = self.create_subscription()
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.pub_trajectory.publish(self.msg)


def main(args=None):
    node = None
    try:
        rclpy.init()
        node = AstarPythonNode(trajectories[0][0][0])
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":

    with open(os.path.dirname(__file__)+"/result/costmap.txt", "rb") as f:
        costmap = pickle.load(f)

    # -- Start and Goal Pose
    start_pose = Pose()

    goal_pose = Pose()

    xs = [i for i in range(-35,36)]
    ys = [i for i in range(-35,36)]
    yaws = [2*np.pi*i/10 for i in range(10)]

    results = np.zeros((len(yaws),len(xs),len(ys)))

    trajectories = [[[Trajectory() for k in range(len(ys))] for j in range(len(xs))] for i in range(len(yaws))]

    ## -- Search
    astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)
    astar.setMap(costmap)

    start_time = time.monotonic()

    for i, yaw in enumerate(tqdm(yaws)):
    # for i, yaw in enumerate([0]):
        for j, x in enumerate(xs):
            for k, y in enumerate(ys):
    # for i, yaw in enumerate([0]):
    #     print("yaw = ", yaw)
    #     for j, x in enumerate([15]):
    #         for k, y in enumerate([20]):
                
                # initialize astar instance every time.
                # astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)
                # astar.setMap(costmap)
                
                goal_pose.position.x = float(x)
                goal_pose.position.y = float(y)

                quaterinon = Quaternion(axis=[0, 0, 1], angle=yaw)

                goal_pose.orientation.w = quaterinon.w
                goal_pose.orientation.x = quaterinon.x
                goal_pose.orientation.y = quaterinon.y
                goal_pose.orientation.z = quaterinon.z
                
                result = astar.makePlan(start_pose, goal_pose)
                results[i][j][k] = result

                if result:
                    waypoints = astar.getWaypoints()
                    trajectories[i][j][k] = createTrajectory(waypoints)


    end_time = time.monotonic()
    print('search_time : ', end_time-start_time)

    result_bag = resultBag(xs, ys, yaws, results, trajectories)
    filename = os.path.dirname(__file__)+"/result/searched_trajectories_with_obstacle_yaw0.txt"
    file1 = open(filename, "wb")
    pickle.dump(result_bag, file1)
    file1.close

    # with open(filename, "rb") as f:
    #     result_bag2 = pickle.load(f)

    # xs = result_bag2.xs
    # ys = result_bag2.ys
    # yaws = result_bag2.yaws
    # results = result_bag2.results
    # trajectories = result_bag2.trajectories
    # # main()

    # ## plot result
    # fig, axes = plt.subplots(nrows=2, ncols=5, figsize=(20,8), tight_layout=True)

    # for i, yaw in enumerate(yaws):
    #     ax=axes[i//5, i%5]
    #     sns.heatmap(results[i], ax=ax, vmin=0, vmax=1, linewidths=1, cbar = False,\
    #                 xticklabels=ys, yticklabels=xs)
    #     for j in range(len(xs)):
    #         for k in range(len(ys)):
    #             ax.plot([point.pose.position.x for point in trajectories[i][j][k].points], \
    #                     [point.pose.position.y for point in trajectories[i][j][k].points])
    #     ax.set_title('yaw = '+str(yaw))
    #     ax.set_ylabel('x')
    #     ax.set_xlabel('y')

    # plt.show()
