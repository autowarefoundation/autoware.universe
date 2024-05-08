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

from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import matplotlib.widgets as wg
from nav_msgs.msg import OccupancyGrid

sys.path.append(os.path.dirname(__file__))

COSTMAP_FILENAME = "dummy"


class DrawRectangleObject:
    def __init__(self, fig, costmap):
        self.press_point = None
        # Initialize a set to keep track of clicked cells
        self.figure = fig
        self.costmap = costmap
        self.x_range = -costmap.info.origin.position.x
        self.y_range = -costmap.info.origin.position.y
        self.obstacles = set()

    def connect(self):
        # connect to all the events we need
        self.cidpress = self.figure.canvas.mpl_connect("button_press_event", self.on_press)
        self.cidrelease = self.figure.canvas.mpl_connect("button_release_event", self.on_release)

    def disconnect(self):
        # disconnect all the stored connection ids
        self.figure.canvas.mpl_disconnect(self.cidpress)
        self.figure.canvas.mpl_disconnect(self.cidrelease)

    def on_press(self, event):
        press_x, press_y = math.floor(event.xdata + 0.5), math.floor(event.ydata + 0.5)
        self.press_point = (press_x, press_y)

    def on_release(self, event):
        if self.press_point is None:
            return

        # Get the clicked coordinates and adjust to the center of the cell
        release_x, release_y = math.floor(event.xdata + 0.5), math.floor(event.ydata + 0.5)
        press_x, press_y = self.press_point[0], self.press_point[1]

        obstacle = (
            press_x,
            press_y,
            release_x - press_x,
            release_y - press_y,
        )  # obstacle=[originx, originy, height, width]
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
        ax1.set_xlim(-self.x_range, self.x_range)
        ax1.set_ylim(-self.y_range, self.y_range)
        ax1.set_xticks(range(-int(self.x_range), int(self.x_range) + 1))
        ax1.set_yticks(range(-int(self.y_range), int(self.y_range) + 1))
        ax1.annotate(
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
        ax1.grid(True)

        # Draw all paths
        rectangles = [
            Rectangle(obstacle[0:2], obstacle[2], obstacle[3]) for obstacle in self.obstacles
        ]
        pc = PatchCollection(rectangles, facecolor="black")
        ax1.add_collection(pc)

        # Redraw the figure to show changes
        plt.draw()

    # TODO
    def generate_costmap(self):
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        for obstacle in self.obstacles:
            originx_idx = int((obstacle[0] + self.x_range) / resolution)
            originy_idx = int((obstacle[1] + self.y_range) / resolution)
            height_idx = int(obstacle[2] / resolution)
            width_idx = int(obstacle[3] / resolution)

            for i in (
                range(originy_idx, originy_idx + width_idx)
                if width_idx > 0
                else range(originy_idx + width_idx, originy_idx)
            ):
                for j in (
                    range(originx_idx, originx_idx + height_idx)
                    if height_idx > 0
                    else range(originx_idx + height_idx, originx_idx)
                ):
                    self.costmap.data[i * width + j] = 100

        return self.costmap


def btn_click(event):
    costmap = draw_rectangle_object.generate_costmap()
    with open(COSTMAP_FILENAME, "wb") as file1:
        pickle.dump(costmap, file1)
        print("costmap is saved at: " + COSTMAP_FILENAME)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="costmap info")
    parser.add_argument(
        "--save_name",
        default="costap_generated",
        type=str,
        help="file name without extension to save",
    )
    parser.add_argument("--height", default=350, type=int, help="height of costmap")
    parser.add_argument("--width", default=350, type=int, help="width of costmap")
    parser.add_argument("--resolution", default=0.2, type=float, help="resolution of costmap")
    args = parser.parse_args()

    COSTMAP_FILENAME = os.path.dirname(__file__) + "/costmap/" + args.save_name + ".txt"

    # -- Costmap Definition
    x_range = args.height * args.resolution / 2
    y_range = args.width * args.resolution / 2

    costmap = OccupancyGrid()
    costmap.info.resolution = args.resolution
    costmap.info.height = args.height
    costmap.info.width = args.width
    costmap.info.origin.position.x = -x_range
    costmap.info.origin.position.y = -y_range
    costmap.data = [0 for i in range(args.height * args.width)]

    # Create a figure and axis to plot the grid
    fig = plt.figure(layout="constrained", figsize=(13, 14))
    subfigs = fig.subfigures(2, 1, width_ratios=[1], height_ratios=[13, 1])

    ax1 = subfigs[0].subplots()
    ax1.set_xlim(-x_range, x_range)
    ax1.set_ylim(-y_range, y_range)
    ax1.set_xticks(range(-int(x_range), int(x_range) + 1))
    ax1.set_yticks(range(-int(y_range), int(y_range) + 1))
    ax1.annotate(
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
    ax1.grid(True)

    # Connect the click event to the drawing function
    ax2 = subfigs[1].subplots()
    btn = wg.Button(ax2, "Generate Costmap", color="#f5f5f5", hovercolor="#a9a9a9")
    btn.on_clicked(btn_click)

    draw_rectangle_object = DrawRectangleObject(subfigs[0], costmap)
    draw_rectangle_object.connect()

    plt.show()
