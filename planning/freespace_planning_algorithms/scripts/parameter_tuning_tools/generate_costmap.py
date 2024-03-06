import math
import rclpy
from nav_msgs.msg import OccupancyGrid
import astar_search_python

class GenerateCostmap:

    def __init__(self, param1):
        self.param1 = param1
        self.costmaps = []
        self.index = 0

    def generate(self):
        return
    
    def generate_simplemap(self, width, height, resolution, n_padding):
        costmap_msg = OccupancyGrid()

        # create info
        costmap_msg.info.width = width
        costmap_msg.info.height = height
        costmap_msg.info.resolution = resolution

        # create data
        n_elem = width * height
        for i in range(n_elem):
            costmap_msg.data.append(0)

        # for i in range(n_padding):
        #     # fill left
        #     for j in range(width * i, width * (i + 1) + 1):
        #         costmap_msg.data[j] = 100.0
        #     # fill right
        #     for j in rage(width * (height - n_padding + i), width * (height - n_padding + i + 1) + 1):
        #         costmap_msg.data[j] = 100.0

        # for i in range(0, height):
        #     # fill bottom
        #     for j in range(i * width, i * width + n_padding + 1):
        #         costmap_msg.data[j] = 100.0
        #     # fill top
        #     for j in range((i + 1) * width - n_padding, (i + 1) * width + 1):
        #         costmap_msg.data[j] = 100.0

        return costmap_msg
    
    def next_costmap(self):
        return self.costmaps[index]

if __name__=='__main__':
    param1 = 1
    generate_costmap = GenerateCostmap(param1)
    # print(generate_costmap.generate_simplemap(150, 150, 0.2, 10))