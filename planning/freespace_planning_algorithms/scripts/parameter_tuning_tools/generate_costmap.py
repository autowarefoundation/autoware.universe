from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as wg
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
import seaborn as sns
import pickle

import sys
import os
sys.path.append(os.path.dirname(__file__))
from common.common_classes import resultBag

from nav_msgs.msg import OccupancyGrid


COSTMAP_FILENAME = os.path.dirname(__file__)+"/result/costmap.txt"

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

        obstacle = (press_x, press_y, release_x-press_x, release_y-press_y) # obstacle=[originx, originy, height, width]
        # Add the cell to the set if it's not there, remove if it is
        if obstacle in self.obstacles:
            self.obstacles.remove(obstacle)
        else:
            self.obstacles.add(obstacle)
        
        self.drawnow()
        self.press_point = None

    def drawnow(self):
        # Clear previous paths and redraw the grid
        ax1.cla()
        plt.draw()
        ax1.set_xlim(-35, 35)
        ax1.set_ylim(-35, 35)
        ax1.set_xticks(range(-35,36))
        ax1.set_yticks(range(-35,36))
        ax1.annotate('', xy=[2, 0], xytext=[0, 0],
                        arrowprops=dict(shrink=0, width=1, headwidth=8, 
                                        headlength=10, connectionstyle='arc3',
                                        facecolor='gray', edgecolor='gray')
                    )
        ax1.grid(True)

        # Draw all paths
        rectangles = [Rectangle(obstacle[0:2], obstacle[2], obstacle[3]) for obstacle in self.obstacles]
        pc = PatchCollection(rectangles, facecolor='black')
        ax1.add_collection(pc)

        # Redraw the figure to show changes
        plt.draw()

    # TODO    
    def generate_costmap(self):
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        for obstacle in self.obstacles:
            originx_idx = int( (obstacle[0]+35) / resolution)
            originy_idx = int( (obstacle[1]+35) / resolution)
            height_idx = int( obstacle[2] / resolution)
            width_idx = int( obstacle[3] / resolution)

            for i in (range(originy_idx, originy_idx + width_idx) if width_idx > 0 \
                        else range(originy_idx + width_idx, originy_idx)):
                for j in (range(originx_idx, originx_idx + height_idx) if height_idx > 0 \
                      else range(originx_idx + height_idx, originx_idx)):

                    self.costmap.data[i*width+j] = 100
        
        return self.costmap


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
fig = plt.figure(layout='constrained', figsize=(13, 14))
subfigs = fig.subfigures(2, 1, width_ratios=[1], height_ratios=[13, 1])

ax1 = subfigs[0].subplots()
ax1.set_xlim(-35, 35)
ax1.set_ylim(-35, 35)
ax1.set_xticks(range(-35,36))
ax1.set_yticks(range(-35,36))
ax1.annotate('', xy=[2, 0], xytext=[0, 0],
                arrowprops=dict(shrink=0, width=1, headwidth=8, 
                                headlength=10, connectionstyle='arc3',
                                facecolor='gray', edgecolor='gray')
               )
ax1.grid(True)

# Connect the click event to the drawing function
draw_rectangle_object = DrawRectangleObject(subfigs[0], costmap)

ax2 = subfigs[1].subplots()
btn = wg.Button(ax2, 'Generate Costmap', color='#f5f5f5', hovercolor='#a9a9a9')

def btn_click(event):
    costmap = draw_rectangle_object.generate_costmap()
    with open(COSTMAP_FILENAME, "wb") as file1:
        pickle.dump(costmap, file1)
        print("costmap is saved at: "+COSTMAP_FILENAME)

btn.on_clicked(btn_click)
draw_rectangle_object.connect()

plt.show()
