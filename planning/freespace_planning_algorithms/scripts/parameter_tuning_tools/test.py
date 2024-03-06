import freespace_planning_algorithms.freespace_planning_algorithms_python as fp
import yaml

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from rclpy.serialization import serialize_message

vehicle_shape = fp.VehicleShape(3, 1, 1)
astar_param = fp.AstarParam()
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
astar_param.only_behind_solutions= False
astar_param.use_back = True
astar_param.distance_heuristic_weight= 1.0

astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)

with open('param/costmap.yml', 'r') as yml:
    costmap_yml = yaml.safe_load(yml)

costmap = OccupancyGrid()
costmap.info.resolution = costmap_yml['info']['resolution']
costmap.info.height = costmap_yml['info']['height']
costmap.info.width = costmap_yml['info']['width']
costmap.info.origin.position.x = costmap_yml['info']['origin']['position']['x']
costmap.info.origin.position.y = costmap_yml['info']['origin']['position']['y']
costmap.info.origin.position.z = costmap_yml['info']['origin']['position']['z']
costmap.data = costmap_yml['data']
costmap_byte = serialize_message(costmap)

astar.setMap(costmap_byte)


start_pose = Pose()
goal_pose = Pose()

with open('param/initialpose.yml', 'r') as yml:
    initial_yml = yaml.safe_load(yml)
with open('param/goalpose.yml', 'r') as yml:
    goal_yml = yaml.safe_load(yml)

start_pose.position.x = initial_yml['pose']['pose']['position']['x']
start_pose.position.y = initial_yml['pose']['pose']['position']['y']
start_pose.position.z = initial_yml['pose']['pose']['position']['z']
start_pose.orientation.z = initial_yml['pose']['pose']['orientation']['z']
start_pose.orientation.w = initial_yml['pose']['pose']['orientation']['w']

goal_pose.position.x = goal_yml['pose']['position']['x']
goal_pose.position.y = goal_yml['pose']['position']['y']
goal_pose.position.z = goal_yml['pose']['position']['z']
goal_pose.orientation.z = goal_yml['pose']['orientation']['z']
goal_pose.orientation.w = goal_yml['pose']['orientation']['w']

start_pose_byte = serialize_message(start_pose)
goal_pose_byte = serialize_message(goal_pose)

print(astar.makePlan(start_pose_byte, goal_pose_byte))