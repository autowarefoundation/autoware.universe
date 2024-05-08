class SearchInfo:
    def __init__(self, costmap, xs, ys, yaws):
        self.costmap = costmap
        self.xs = xs
        self.ys = ys
        self.yaws = yaws


class Result:
    def __init__(self, x, y, yaw, find, trajectory):
        self.xs = x
        self.ys = y
        self.yaws = yaw
        self.find = find
        self.trajectory = trajectory
