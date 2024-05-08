# Copyright 2024 TIER IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import math
import os
import pickle
import sys

from autoware_auto_planning_msgs.msg import Trajectory
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from tqdm import tqdm

sys.path.append(os.path.dirname(__file__))

# colors
TAB_COLORS = mcolors.TABLEAU_COLORS
COLOR_KEYS = list(TAB_COLORS.keys())


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
        # connect to all the events we need
        self.cidpress = self.figure.canvas.mpl_connect("button_press_event", self.on_press)
        self.cidrelease = self.figure.canvas.mpl_connect("button_release_event", self.on_release)

    def on_press(self, event):
        press_x, press_y = event.xdata, event.ydata
        self.press_point = (press_x, press_y)

    def on_release(self, event):
        if self.press_point is None:
            return

        # Get the clicked coordinates and adjust to the center of the cell
        release_x, release_y = int(event.xdata), int(event.ydata)
        yaw = math.atan2(release_y - self.press_point[1], release_x - self.press_point[0]) + (
            2 * math.pi / 20
        )
        if yaw < 0:
            yaw += 2 * math.pi
        # TODO: parameterize
        yaw_d = int(yaw * (10 / (2 * math.pi)))
        cell_center = (
            math.floor(self.press_point[0] + 0.5),
            math.floor(self.press_point[1] + 0.5),
            yaw_d,
        )  # Center of the clicked cell

        # Add the cell to the set if it's not there, remove if it is
        if cell_center in self.clicked_cells:
            self.clicked_cells.remove(cell_center)
        else:
            self.clicked_cells.add(cell_center)

        self.drawnow()
        self.press_point = None

    def disconnect(self):
        # disconnect all the stored connection ids
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
        ax.annotate(
            "",
            xy=[2, 0],
            xytext=[0, 0],
            arrowprops={
                "shrink": 0,
                "width": 1,
                "headwidth": 8,
                "headlength": 10,
                "connectionstyle": "arc3",
                "facecolor": "gray",
                "edgecolor": "gray",
            },
        )
        ax.scatter(obstacle_x, obstacle_y, c="#000000", s=1)
        ax.grid(True)

        # Draw all paths
        for c, cell in enumerate(self.clicked_cells):
            i = self.yaws.index(cell[2])
            j = self.xs.index(cell[0])
            k = self.ys.index(cell[1])

            color_key = COLOR_KEYS[c % len(TAB_COLORS)]
            color = TAB_COLORS[color_key]

            # plot path
            ax.plot(
                [point.pose.position.x for point in self.trajectories[i][j][k].points],
                [point.pose.position.y for point in self.trajectories[i][j][k].points],
                color=color,
            )
            # plot goal arrow
            goal = [cell[0], cell[1]]
            yaw = (2 * math.pi / 10) * cell[2]
            direction = [2 * math.cos(yaw) + cell[0], 2 * math.sin(yaw) + cell[1]]
            ax.annotate(
                "",
                xy=direction,
                xytext=goal,
                arrowprops={
                    "shrink": 0,
                    "width": 1,
                    "headwidth": 8,
                    "headlength": 10,
                    "connectionstyle": "arc3",
                    "facecolor": color,
                    "edgecolor": color,
                },
            )
        # Redraw the figure to show changes
        plt.draw()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="place of save result")
    parser.add_argument("--dir_name", default="default_dir", type=str, help="saved directory name")
    args = parser.parse_args()

    save_dir = os.path.dirname(__file__) + "/result/" + args.dir_name

    # laod search settings
    with open(save_dir + "/info.txt", "rb") as f:
        info = pickle.load(f)
        costmap = info.costmap
        x_range = -costmap.info.origin.position.x
        y_range = -costmap.info.origin.position.y
        xs = info.xs
        ys = info.ys
        yaws = info.yaws
        yaws_d = [int((10 / (2 * math.pi)) * yaw) for yaw in yaws]

    # load results
    results = np.zeros((len(yaws), len(xs), len(ys)))
    trajectories = [
        [[Trajectory() for k in range(len(ys))] for j in range(len(xs))] for i in range(len(yaws))
    ]

    print("loading result datas...")
    for i, x in enumerate(tqdm(xs)):
        for j, y in enumerate(ys):
            for k, yaw in enumerate(yaws):
                filename = save_dir + "/" + str(x) + "_" + str(y) + "_" + str(k) + ".txt"
                with open(filename, "rb") as f:
                    result = pickle.load(f)
                    results[k][i][j] = result.find
                    trajectories[k][i][j] = result.trajectory

    # detect obstacle
    obstacle_x = []
    obstacle_y = []
    for i, cost in enumerate(costmap.data):
        if cost >= 100:
            obstacle_x.append((i % costmap.info.width) * costmap.info.resolution - x_range)
            obstacle_y.append((i / costmap.info.width) * costmap.info.resolution - y_range)

    # plot result
    fig_result, axes_result = plt.subplots(nrows=2, ncols=5, figsize=(25, 10), tight_layout=True)
    fig_result.suptitle("Search result, [white: success, red: failed, black: obstacle]")

    for i, yaw in enumerate(yaws):
        result = np.transpose(results[i])

        # plot result
        ax_result = axes_result[i // 5, i % 5]
        sns.heatmap(
            result,
            ax=ax_result,
            cmap="Reds_r",
            vmin=-1,
            vmax=1,
            center=0,
            linewidths=0.5,
            cbar=False,
            xticklabels=xs,
            yticklabels=ys,
        )
        # plot obstacle
        ax_result.scatter(
            np.array(obstacle_x) + x_range + 0.5,
            np.array(obstacle_y) + y_range + 0.5,
            c="black",
            s=1,
        )

        ax_result.invert_yaxis()
        ax_result.set_title("yaw = " + str(yaw))
        ax_result.set_xlabel("x")
        ax_result.set_ylabel("y")

    plt.draw()

    # plot path
    fig, ax = plt.subplots(figsize=(13, 13))
    ax.set_xlim([xs[0], xs[-1]])
    ax.set_ylim([ys[0], ys[-1]])
    ax.set_xticks(xs)
    ax.set_yticks(ys)
    ax.annotate(
        "",
        xy=[2, 0],
        xytext=[0, 0],
        arrowprops={
            "shrink": 0,
            "width": 1,
            "headwidth": 8,
            "headlength": 10,
            "connectionstyle": "arc3",
            "facecolor": "gray",
            "edgecolor": "gray",
        },
    )
    ax.scatter(obstacle_x, obstacle_y, c="#000000", s=1)
    ax.grid(True)

    # Connect the click event to the drawing function
    draw_clicked_trajectory = DrawClickedTrajectory(fig, xs, ys, yaws_d, trajectories)
    draw_clicked_trajectory.connect()

    plt.show()
