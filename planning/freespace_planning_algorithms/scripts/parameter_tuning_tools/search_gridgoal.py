import freespace_planning_algorithms.astar_search as fp

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.serialization import serialize_message

from autoware_auto_planning_msgs.msg import Trajectory

import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd
from pyquaternion import Quaternion
import time


# -- Vehicle Shape -- 
vehicle_shape = fp.VehicleShape()
vehicle_shape.length = 4.89
vehicle_shape.width = 1.896
vehicle_shape.base2back = 1.1


# -- Planner Common Parameter --
planner_param = fp.PlannerCommonParam()
# base configs
planner_param.time_limit= 30000.0
planner_param.minimum_turning_radius= 9.0
planner_param.maximum_turning_radius= 9.0
planner_param.turning_radius_size= 1
# search configs
planner_param.theta_size= 144
planner_param.angle_goal_range= 6.0
planner_param.curve_weight= 1.2
planner_param.reverse_weight= 2.0
planner_param.lateral_goal_range= 0.5
planner_param.longitudinal_goal_range= 2.0
# costmap configs
planner_param.obstacle_threshold= 100


# -- A* search Configurations --
astar_param = fp.AstarParam()
astar_param.only_behind_solutions= False
astar_param.use_back = True
astar_param.distance_heuristic_weight= 1.0
# astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)


# -- Costmap Definition
size = 350 #110
resolution = 0.2

costmap = OccupancyGrid()
costmap.info.resolution = resolution
costmap.info.height = size
costmap.info.width = size
costmap.info.origin.position.x = -size*resolution/2
costmap.info.origin.position.y = -size*resolution/2
costmap.data = [0 for i in range(size**2) ]


# -- Start and Goal Pose
start_pose = Pose()

goal_pose = Pose()

yaws = [2*np.pi*i/10 for i in range(10)]
xs = [i for i in range(-35,36)]
ys = [i for i in range(-35,36)]

results = np.zeros((len(yaws),len(xs),len(ys)))

# -- Trajectory
class Trajectory:
    def __init__(self, x, y):
        self.x = x
        self.y = y

trajectories = [[[Trajectory(0,0) for i in range(len(ys))] for i in range(len(xs))] for i in range(len(yaws))]

## search grid goal
start_time = time.monotonic()

# for i, yaw in enumerate(yaws):
for i, yaw in enumerate([0]):
    # print("yaw = ", yaw)
    for j, x in enumerate(xs):
        for k, y in enumerate(ys):
# for i, yaw in enumerate([0]):
#     print("yaw = ", yaw)
#     for j, x in enumerate([15]):
#         for k, y in enumerate([20]):
            
            # initialize astar instance every time.
            astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)
            astar.setMap(costmap)
            
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
                trajectories[i][j][k] = Trajectory([waypoint.position.x+35.0+0.5 for waypoint in waypoints.waypoints], [waypoint.position.y+35.0+0.5 for waypoint in waypoints.waypoints])


end_time = time.monotonic()
print('search_time : ', end_time-start_time)

# plot result
fig, axes = plt.subplots(nrows=2, ncols=5, figsize=(20,8), tight_layout=True)

for i, yaw in enumerate(yaws):
    ax=axes[i//5, i%5]
    sns.heatmap(results[i], ax=ax, vmin=0, vmax=1, linewidths=1, cbar = False,\
                xticklabels=ys, yticklabels=xs)
    for j in range(len(xs)):
        for k in range(len(ys)):
            ax.plot(trajectories[i][j][k].y, trajectories[i][j][k].x)
    ax.set_title('yaw = '+str(yaw))
    ax.set_ylabel('x')
    ax.set_xlabel('y')

plt.show()