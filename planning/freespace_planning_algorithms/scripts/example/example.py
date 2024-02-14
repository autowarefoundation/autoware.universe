import freespace_planning_algorithms.freespace_planning_algorithms_python as fp

from pyquaternion import Quaternion

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from rclpy.serialization import serialize_message


# -- Vehicle Shape -- 
vehicle_shape = fp.VehicleShape()
vehicle_shape.length = 2.0
vehicle_shape.width = 1.0
vehicle_shape.base2back = 1.0


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

astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)


# -- Costmap Definition
size = 350
resolution = 0.2

costmap = OccupancyGrid()
costmap.info.resolution = resolution
costmap.info.height = size
costmap.info.width = size
costmap.info.origin.position.x = -size*resolution/2
costmap.info.origin.position.y = -size*resolution/2
costmap.data = [0 for i in range(size**2) ]

astar.setMap(serialize_message(costmap))


# -- Start and Goal Pose
start_pose = Pose()
goal_pose = Pose()

start_pose.position.x = 0.0
start_pose.position.y = 0.0

goal_pose.position.x = 10.0
goal_pose.position.y = 0.0

yaw = 0
quaterinon = Quaternion(axis=[0, 0, 1], angle=yaw)

goal_pose.orientation.w = quaterinon.w
goal_pose.orientation.x = quaterinon.x
goal_pose.orientation.y = quaterinon.y
goal_pose.orientation.z = quaterinon.z


# -- Search --
if astar.makePlan(serialize_message(start_pose),\
                  serialize_message(goal_pose)):
    print('Success to find path.')
else:
    print('Fail to find path.')