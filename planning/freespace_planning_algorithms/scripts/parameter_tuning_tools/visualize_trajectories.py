from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint

import math
import matplotlib.pyplot as plt
import seaborn as sns
import pickle

import sys
import os
sys.path.append(os.path.dirname(__file__))
from common.common_classes import resultBag


class DrawClickedTrajectory:
    def __init__(self, fig, xs, ys, yaws, trajectories):
        self.press_point = None
        # Initialize a set to keep track of clicked cells
        self.figure = fig
        self.xs = xs
        self.ys = ys
        self.yaws = yaws
        self.trajectories = trajectories
        self.clicked_cells = set()
        self.cell_center = None

    def connect(self):
        'connect to all the events we need'
        self.cidpress = self.figure.canvas.mpl_connect(
            'button_press_event', self.on_press)
        self.cidrelease = self.figure.canvas.mpl_connect(
            'button_release_event', self.on_release)
        
    def on_press(self, event):
        press_x, press_y = int(event.xdata), int(event.ydata)
        self.press_point = (press_x + 0.5, press_y + 0.5)
        
    def on_release(self, event):
        if self.press_point == None:
            return
        
        # Get the clicked coordinates and adjust to the center of the cell
        release_x, release_y = int(event.xdata), int(event.ydata)
        yaw = math.atan2(release_y-self.press_point[1],release_x-self.press_point[0])
        if yaw < 0:
            yaw += 2*math.pi
        # TODO: parameterize
        yaw_d = int(yaw*(10/(2*math.pi)))
        cell_center = (self.press_point[0] + 0.5, self.press_point[1] + 0.5, yaw_d)  # Center of the clicked cell

        # Add the cell to the set if it's not there, remove if it is
        if cell_center in self.clicked_cells:
            self.clicked_cells.remove(cell_center)
        else:
            self.clicked_cells.add(cell_center)
        
        self.drawnow()
        self.press_point = None

    def disconnect(self):
        'disconnect all the stored connection ids'
        self.figure.canvas.mpl_disconnect(self.cidpress)
        self.figure.canvas.mpl_disconnect(self.cidrelease)

    def drawnow(self):
        # Clear previous paths and redraw the grid
        ax.cla()
        plt.draw()
        ax.set_xlim([self.xs[0], self.xs[-1]])
        ax.set_ylim([self.ys[0], self.ys[-1]])
        ax.set_xticks(self.xs)
        ax.set_yticks(self.ys)
        ax.annotate('', xy=[2, 0], xytext=[0, 0],
                arrowprops=dict(shrink=0, width=1, headwidth=8, 
                                headlength=10, connectionstyle='arc3',
                                facecolor='gray', edgecolor='gray')
               )
        ax.grid(True)

        # Center cell coordinates
        center_x, center_y = 5, 5

        # Draw all paths
        for cell in self.clicked_cells:
            i = self.yaws.index(cell[2])
            j = self.xs.index(cell[0])
            k = self.ys.index(cell[1])
            x = [point.pose.position.x for point in self.trajectories[i][j][k].points]
            y = [point.pose.position.y for point in self.trajectories[i][j][k].points]
            ax.plot([point.pose.position.x for point in self.trajectories[i][j][k].points], \
                    [point.pose.position.y for point in self.trajectories[i][j][k].points])

            goal = [cell[0], cell[1]]
            yaw = (2*math.pi/10)*cell[2]
            direction = [2*math.cos(yaw)+cell[0], 2*math.sin(yaw)+cell[1]]
            ax.annotate('', xy=direction, xytext=goal,
                arrowprops=dict(shrink=0, width=1, headwidth=8, 
                                headlength=10, connectionstyle='arc3',
                                facecolor='red', edgecolor='red')
               )
        # Redraw the figure to show changes
        plt.draw()


with open(os.path.dirname(__file__)+"/result/searched_trajectories.txt", "rb") as f:
    result_bag = pickle.load(f)

xs = result_bag.xs
ys = result_bag.ys
yaws = result_bag.yaws
yaws_d = [int((10/(2*math.pi))*yaw) for yaw in yaws]
results = result_bag.results
trajectories = result_bag.trajectories

# Create a figure and axis to plot the grid
fig, ax = plt.subplots()
ax.set_xlim([xs[0], xs[-1]])
ax.set_ylim([ys[0], ys[-1]])
ax.set_xticks(xs)
ax.set_yticks(ys)
ax.annotate('', xy=[2, 0], xytext=[0, 0],
                arrowprops=dict(shrink=0, width=1, headwidth=8, 
                                headlength=10, connectionstyle='arc3',
                                facecolor='gray', edgecolor='gray')
               )
ax.grid(True)

# Connect the click event to the drawing function
draw_clicked_trajectory = DrawClickedTrajectory(fig, xs, ys, yaws_d, trajectories)
draw_clicked_trajectory.connect()
# fig.canvas.mpl_connect("button_press_event", draw_paths)

plt.show()
