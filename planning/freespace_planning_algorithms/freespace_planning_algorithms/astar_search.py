import freespace_planning_algorithms.freespace_planning_algorithms_pybind as _fp
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import serialize_message

PlannerCommonParam = _fp.PlannerCommonParam
VehicleShape = _fp.VehicleShape
AstarParam = _fp.AstarParam


class PlannerWaypoints:
    def __init__(self):
        self.waypoints = []
        self.length = 0.0

    def compute_length(self):
        return self.length


class AstarSearch:
    def __init__(
        self,
        planner_param: PlannerCommonParam,
        vehicle_shape: VehicleShape,
        astar_param: AstarParam,
    ):
        self.astar_search = _fp.AstarSearch(planner_param, vehicle_shape, astar_param)

    def setMap(self, costmap: OccupancyGrid):
        costmap_byte = serialize_message(costmap)
        self.astar_search.setMap(costmap_byte)

    def makePlan(self, start_pose: Pose, goal_pose: Pose):
        start_pose_byte = serialize_message(start_pose)
        goal_pose_byte = serialize_message(goal_pose)
        return self.astar_search.makePlan(start_pose_byte, goal_pose_byte)

    def getWaypoints(self):
        waypoints_vetor = self.astar_search.getWaypoints()
        waypoints = PlannerWaypoints()

        waypoints.length = waypoints_vetor.length
        for waypoint in waypoints_vetor.waypoints:
            pos = Point(x=waypoint[0], y=waypoint[1], z=waypoint[2])
            quat = Quaternion(x=waypoint[3], y=waypoint[4], z=waypoint[5], w=waypoint[6])
            waypoints.waypoints.append(Pose(position=pos, orientation=quat))

        return waypoints
