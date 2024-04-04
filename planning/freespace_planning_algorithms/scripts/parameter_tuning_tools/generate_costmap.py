from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
import seaborn as sns
import pickle

import sys
import os
sys.path.append(os.path.dirname(__file__))
from common.common_classes import resultBag

from nav_msgs.msg import OccupancyGrid

class DrawRectangleObject:
    def __init__(self, fig, costmap):
        self.press_point = None
        # Initialize a set to keep track of clicked cells
        self.figure = fig
        self.costmap = costmap
        self.obstacles = set()

    def connect(self):
        'connect to all the events we need'
        self.cidpress = self.figure.canvas.mpl_connect(
            'button_press_event', self.on_press)
        self.cidrelease = self.figure.canvas.mpl_connect(
            'button_release_event', self.on_release)
        
    def disconnect(self):
        'disconnect all the stored connection ids'
        self.figure.canvas.mpl_disconnect(self.cidpress)
        self.figure.canvas.mpl_disconnect(self.cidrelease)
    
    def on_press(self, event):
        press_x, press_y = math.floor(event.xdata+0.5), math.floor(event.ydata+0.5)
        self.press_point = (press_x, press_y)

    def on_release(self, event):
        if self.press_point == None:
            return
        
        # Get the clicked coordinates and adjust to the center of the cell
        release_x, release_y = math.floor(event.xdata+0.5), math.floor(event.ydata+0.5)
        press_x, press_y = self.press_point[0], self.press_point[1]

        obstacle = (press_x, press_y, release_x-press_x, release_y-press_y) # obstacle=[originx, originy, width, height]
        # Add the cell to the set if it's not there, remove if it is
        if obstacle in self.obstacles:
            self.obstacles.remove(obstacle)
        else:
            self.obstacles.add(obstacle)
        
        self.drawnow()
        self.press_point = None

    def drawnow(self):
        # Clear previous paths and redraw the grid
        ax.cla()
        plt.draw()
        ax.set_xlim(-35, 35)
        ax.set_ylim(-35, 35)
        ax.set_xticks(range(-35,36))
        ax.set_yticks(range(-35,36))
        ax.annotate('', xy=[2, 0], xytext=[0, 0],
                        arrowprops=dict(shrink=0, width=1, headwidth=8, 
                                        headlength=10, connectionstyle='arc3',
                                        facecolor='gray', edgecolor='gray')
                    )
        ax.grid(True)

        # Draw all paths
        rectangles = [Rectangle(obstacle[0:2], obstacle[2], obstacle[3]) for obstacle in self.obstacles]
        pc = PatchCollection(rectangles, facecolor='black')
        ax.add_collection(pc)

        # Redraw the figure to show changes
        plt.draw()

    # TODO    
    # def generate_costmap(self):
    #     for obstacle in self.obstacles:



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


# Create a figure and axis to plot the grid
fig, ax = plt.subplots()
ax.set_xlim(-35, 35)
ax.set_ylim(-35, 35)
ax.set_xticks(range(-35,36))
ax.set_yticks(range(-35,36))
ax.annotate('', xy=[2, 0], xytext=[0, 0],
                arrowprops=dict(shrink=0, width=1, headwidth=8, 
                                headlength=10, connectionstyle='arc3',
                                facecolor='gray', edgecolor='gray')
               )
ax.grid(True)

# Connect the click event to the drawing function
draw_rectangle_object = DrawRectangleObject(fig, costmap)
draw_rectangle_object.connect()
# fig.canvas.mpl_connect("button_press_event", draw_paths)

plt.show()
