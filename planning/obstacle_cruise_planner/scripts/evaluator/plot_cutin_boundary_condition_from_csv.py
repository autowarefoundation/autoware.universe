#!/usr/bin/env python3

# Copyright 2022 Tier IV, Inc.
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
import csv
import math
import webbrowser

from matplotlib import colors
from matplotlib.backend_bases import MouseButton
import matplotlib.pyplot as plt
import numpy as np


def is_avoidance_admitted(ego_vel, obj_vel, cutin_dist):
    lc_lon_offset = 20.0
    lc_lat_offset = 2.0
    time_to_intrude = math.hypot(lc_lon_offset, lc_lat_offset) / obj_vel

    time_thresh = (ego_vel - obj_vel) / (2 * 6) + 0.35 + time_to_intrude

    return cutin_dist / (ego_vel - obj_vel) < time_thresh


def mps2kmph(val):
    return round(val * 3.6, 1)


def on_click(event, im, plot_url, min_x, max_x, interval_x, min_y, max_y, interval_y):

    if event.button is not MouseButton.LEFT:
        return
    if not im.contains(event):
        return

    x = event.xdata
    y = event.ydata
    if x is None or y is None:
        return

    rel_x = x - min_x + interval_x / 2
    rel_y = y - min_y + interval_y / 2

    sample_x = (max_x - min_x) / interval_x
    sample_y = (max_y - min_y) / interval_y

    modified_interval_x = round((max_x - min_x) / sample_x)
    modified_interval_y = round((max_y - min_y) / sample_y)

    x_idx = int(round(rel_x // modified_interval_x))
    y_idx = int(round(rel_y // modified_interval_y))

    webbrowser.open(plot_url[len(plot_url) - y_idx - 1][x_idx])


def plot(file_path):

    # parameters
    ego_vel = 11.1
    variables = ["obj_vel", "cutin_dist"]

    # read csv and create raw data
    raw_data_dict = {}
    with open(file_path) as f:
        f = csv.DictReader(f)
        for row in f:
            params = row["parameters"]
            status = 1 if row["status"] == "succeeded" else 0
            url = row["detail_url"]

            # create data for plotting
            raw_data = []
            for variable in variables:
                splitted_params = params.split("tier4_modifier_" + variable + "=")
                if len(splitted_params) == 1:
                    print(splitted_params)
                    print("Error occurred. Continue.")
                    continue
                param = splitted_params[1].split("__")[0]
                raw_data.append(float(param))

            raw_data_dict[tuple(raw_data)] = [status, url]

    # create actual data to plot
    unique_variables = []
    for v_idx in range(len(variables)):
        variable_fixed_data = [raw_data[v_idx] for raw_data in raw_data_dict]
        unique_variables.append(sorted(set(variable_fixed_data)))

    plot_actual_data = []
    plot_url = []
    for i in reversed(unique_variables[0]):
        plot_part_data = []
        plot_part_url = []
        for j in unique_variables[1]:
            plot_part_data.append(raw_data_dict[(i, j)][0])
            plot_part_url.append(raw_data_dict[(i, j)][1])
        plot_actual_data.append(plot_part_data)
        plot_url.append(plot_part_url)

    # create ideal data to plot
    plot_ideal_data = []
    for obj_vel in reversed(unique_variables[0]):
        plot_part_data = []
        for cutin_dist in unique_variables[1]:
            plot_part_data.append(not is_avoidance_admitted(ego_vel, obj_vel, cutin_dist))
        plot_ideal_data.append(plot_part_data)

    # prepare plot
    min_x = min(unique_variables[1])
    max_x = max(unique_variables[1])
    interval_x = min(np.abs(np.diff(np.array(list(unique_variables[1])))))
    min_y = mps2kmph(min(unique_variables[0]))
    max_y = mps2kmph(max(unique_variables[0]))
    interval_y = mps2kmph(min(np.abs(np.diff(np.array(list(unique_variables[0]))))))

    cmap = colors.ListedColormap(["red", "green"])
    bounds = [0, 0.5, 1]
    norm = colors.BoundaryNorm(bounds, cmap.N)

    fig = plt.figure()
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)

    # plot
    im1 = ax1.imshow(
        plot_actual_data,
        extent=(
            min_x - interval_x / 2,
            max_x + interval_x / 2,
            min_y - interval_y / 2,
            max_y + interval_y / 2,
        ),
        cmap=cmap,
        norm=norm,
        aspect=1.5,
    )
    ax2.imshow(
        plot_ideal_data,
        extent=(
            min_x - interval_x / 2,
            max_x + interval_x / 2,
            min_y - interval_y / 2,
            max_y + interval_y / 2,
        ),
        cmap=cmap,
        norm=norm,
        aspect=1.5,
    )

    fig.suptitle("Result of cut-in deceleration (ego velocity is 40kmph)")
    ax1.set_title("Actual (scenario sim)")
    ax2.set_title("Ideal")

    ax1.set_xlabel("cut-in distance [m]")
    ax1.set_ylabel("object velocity [kmph]")
    ax2.set_xlabel("cut-in distance [m]")
    ax2.set_ylabel("object velocity [kmph]")

    ax1.set_xticks(np.arange(min_x, max_x + 0.0001, interval_x))
    ax1.set_yticks(np.arange(min_y, max_y + 0.0001, interval_y))
    ax2.set_xticks(np.arange(min_x, max_x + 0.0001, interval_x))
    ax2.set_yticks(np.arange(min_y, max_y + 0.0001, interval_y))

    # grid
    for x in np.arange(min_x - interval_x / 2, max_x + interval_x / 2 + 0.0001, interval_x):
        ax1.axvline(x, color="k")
        ax2.axvline(x, color="k")
    for y in np.arange(min_y - interval_y / 2, max_y + interval_y / 2 + 0.0001, interval_y):
        ax1.axhline(y, color="k")
        ax2.axhline(y, color="k")

    # insert hyperlink
    plt.connect(
        "button_press_event",
        lambda e: on_click(e, im1, plot_url, min_x, max_x, interval_x, min_y, max_y, interval_y),
    )

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file-path", default="sample.csv", type=str)
    args = parser.parse_args()

    plot(args.file_path)
