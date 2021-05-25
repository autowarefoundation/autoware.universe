#!/usr/bin/env python

# Copyright 2020 Tier IV, Inc.
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
from os import getcwd
from os.path import basename
from os.path import dirname
from os.path import exists
from os.path import join
from os.path import splitext
import subprocess
import sys

import numpy as np

try:
    import pandas as pd
except ImportError:
    print('Please install pandas. See http://pandas.pydata.org/pandas-docs/stable/')
    sys.exit(1)

FREQ_SAMPLE = 0.001

# low pass filter


def lowpass_filter(data, cutoff_freq=2, order=1, dt=FREQ_SAMPLE):
    tau = 1.0 / (2 * np.pi * cutoff_freq)
    for _ in range(order):
        for i in range(1, len(data)):
            data[i] = (tau / (tau + dt) * data[i - 1] +
                       dt / (tau + dt) * data[i])
    return data


def rel2abs(path):
    """Return absolute path from relative path input."""
    return join(getcwd(), path)


def rosbag_to_csv(path, topic_name):
    name = splitext(basename(path))[0]
    suffix = topic_name.replace('/', '-')
    output_path = join(dirname(path), name + '_' + suffix + '.csv')
    if exists(output_path):
        return output_path
    else:
        command = "rostopic echo -b {0} -p /{1} | sed -e 's/,/ /g' > {2}".format(
            path, topic_name, output_path)
        print(command)
        subprocess.check_call(command, shell=True)
        return output_path


def getActValue(df, speed_type):
    tm = np.array(list(df['%time'])) * 1e-9
    # Unit Conversion
    if speed_type:
        val = np.array(list(df['field'])) / 3.6
    else:
        val = np.array(list(df['field']))
    # Calc differential
    d_val = (val[2:] - val[:-2]) / (tm[2:] - tm[:-2])
    return tm[1:-1], val[1:-1], d_val


def getCmdValueWithDelay(df, delay):
    tm = np.array(list(df['%time'])) * 1e-9
    val = np.array(list(df['field']))
    return tm + delay, val


def getLinearInterpolate(_tm, _val, _index, ti):
    tmp_t = _tm[_index]
    tmp_next_t = _tm[_index + 1]
    tmp_val = _val[_index]
    tmp_next_val = _val[_index + 1]
    val_i = tmp_val + (tmp_next_val - tmp_val) / \
        (tmp_next_t - tmp_t) * (ti - tmp_t)
    return val_i


def getFittingTimeConstantParam(cmd_data, act_data,
                                delay, args, speed_type=False):
    tm_cmd, cmd_delay = getCmdValueWithDelay(cmd_data, delay)
    tm_act, act, d_act = getActValue(act_data, speed_type)
    _t_min = max(tm_cmd[0], tm_act[0])
    _t_max = min(tm_cmd[-1], tm_act[-1])
    tm_cmd = tm_cmd - _t_min
    tm_act = tm_act - _t_min
    MAX_CNT = int((_t_max - _t_min - args.cutoff_time) / FREQ_SAMPLE)
    d_act_sample = [None] * MAX_CNT
    diff_act_cmd_sample = [None] * MAX_CNT
    ind_cmd = 0
    ind_act = 0
    for ind in range(MAX_CNT):
        ti = ind * FREQ_SAMPLE + args.cutoff_time
        while (tm_cmd[ind_cmd + 1] < ti):
            ind_cmd += 1
        cmd_delay_i = getLinearInterpolate(tm_cmd, cmd_delay, ind_cmd, ti)
        while (tm_act[ind_act + 1] < ti):
            ind_act += 1
        act_i = getLinearInterpolate(tm_act, act, ind_act, ti)
        d_act_i = getLinearInterpolate(tm_act, d_act, ind_act, ti)
        d_act_sample[ind] = d_act_i
        diff_act_cmd_sample[ind] = act_i - cmd_delay_i
    d_act_sample = np.array(d_act_sample)
    diff_act_cmd_sample = np.array(diff_act_cmd_sample)
    if args.cutoff_freq > 0:
        d_act_sample = lowpass_filter(d_act_sample, cutoff_freq=args.cutoff_freq)
        diff_act_cmd_sample = lowpass_filter(
            diff_act_cmd_sample, cutoff_freq=args.cutoff_freq)
    d_act_sample = d_act_sample.reshape(1, -1)
    diff_act_cmd_sample = diff_act_cmd_sample.reshape(1, -1)
    tau = -np.dot(diff_act_cmd_sample, np.linalg.pinv(d_act_sample))[0, 0]
    error = np.linalg.norm(diff_act_cmd_sample + tau *
                           d_act_sample) / d_act_sample.shape[1]
    return tau, error


def getFittingParam(cmd_data, act_data, args, speed_type=False):
    delay_range = int((args.max_delay - args.min_delay) / args.delay_incr)
    delays = [
        args.min_delay +
        i *
        args.delay_incr for i in range(
            delay_range +
            1)]
    error_min = 1.0e10
    delay_opt = -1
    tau_opt = -1
    for delay in delays:
        tau, error = getFittingTimeConstantParam(
            cmd_data, act_data, delay, args, speed_type=speed_type)
        if tau > 0:
            if error < error_min:
                error_min = error
                delay_opt = delay
                tau_opt = tau
        else:
            break
    return tau_opt, delay_opt, error_min


if __name__ == '__main__':
    topics = ['vehicle_cmd/ctrl_cmd/steering_angle', 'vehicle_status/angle',
              'vehicle_cmd/ctrl_cmd/linear_velocity', 'vehicle_status/speed']
    pd_data = [None] * len(topics)
    parser = argparse.ArgumentParser(
        description='Parameter fitting for Input Delay Model'
        ' (First Order System with Dead Time) with rosbag file input')
    parser.add_argument(
        '--bag_file',
        '-b',
        required=True,
        type=str,
        help='rosbag file',
        metavar='file')
    parser.add_argument(
        '--cutoff_time',
        default=1.0,
        type=float,
        help='Cutoff time[sec], Parameter fitting will only consider data '
        'from t= cutoff_time to the end of the bag file (default is 1.0)')
    parser.add_argument(
        '--cutoff_freq',
        default=0.1,
        type=float,
        help='Cutoff freq for low-pass filter[Hz], '
        'negative value will disable low-pass filter (default is 0.1)')
    parser.add_argument(
        '--min_delay',
        default=0.1,
        type=float,
        help='Min value for searching delay loop (default is 0.1)')
    parser.add_argument(
        '--max_delay',
        default=1.0,
        type=float,
        help='Max value for searching delay loop (default is 1.0)')
    parser.add_argument(
        '--delay_incr',
        default=0.01,
        type=float,
        help='Step value for searching delay loop (default is 0.01)')
    args = parser.parse_args()

    for i, topic in enumerate(topics):
        csv_log = rosbag_to_csv(rel2abs(args.bag_file), topic)
        pd_data[i] = pd.read_csv(csv_log, sep=' ')
    tau_opt, delay_opt, error = getFittingParam(
        pd_data[0], pd_data[1], args, speed_type=False)
    print(
        'Steer angle: tau_opt = %2.4f, delay_opt = %2.4f, error = %2.4e' %
        (tau_opt, delay_opt, error))
    tau_opt, delay_opt, error = getFittingParam(
        pd_data[2], pd_data[3], args, speed_type=True)
    print(
        'Velocity   : tau_opt = %2.4f, delay_opt = %2.4f, error = %2.4e' %
        (tau_opt, delay_opt, error))
