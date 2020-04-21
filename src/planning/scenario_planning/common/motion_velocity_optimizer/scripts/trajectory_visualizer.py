#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import time
from autoware_planning_msgs.msg import Trajectory
import matplotlib.pyplot as plt
import numpy as np

class TrajectoryVisualizer():

    def __init__(self):
        self.trajectory_external_velocity_limitted = Trajectory()
        self.trajectory_lateral_acc_filtered = Trajectory()
        self.trajectory_raw = Trajectory()
        self.trajectory_time_resamped = Trajectory()
        self.trajectory_final = Trajectory()
        self.plot_done1 = True
        self.plot_done2 = True
        self.plot_done3 = True
        self.plot_done4 = True
        self.plot_done5 = True

        self.substatus1 = rospy.Subscriber("/planning/scenario_planning/motion_velocity_optimizer/debug/trajectory_external_velocity_limitted", Trajectory, self.CallBackTrajExVelLim, queue_size=1, tcp_nodelay=True)
        self.substatus2 = rospy.Subscriber("/planning/scenario_planning/motion_velocity_optimizer/debug/trajectory_lateral_acc_filtered", Trajectory, self.CallBackTrajLatAccFiltered, queue_size=1, tcp_nodelay=True)
        self.substatus3 = rospy.Subscriber("/planning/scenario_planning/motion_velocity_optimizer/debug/trajectory_raw", Trajectory, self.CallBackTrajRaw, queue_size=1, tcp_nodelay=True)
        self.substatus4 = rospy.Subscriber("/planning/scenario_planning/motion_velocity_optimizer/debug/trajectory_time_resampled", Trajectory, self.CallBackTrajTimeResampled, queue_size=1, tcp_nodelay=True)
        self.substatus5 = rospy.Subscriber("/planning/scenario_planning/trajectory", Trajectory, self.CallBackTrajFinal, queue_size=1, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(0.3), self.timerCallback)

    def CallBackTrajExVelLim(self, cmd):
        if (self.plot_done1):
            self.trajectory_external_velocity_limitted = cmd
            self.plot_done1 = False

    def CallBackTrajLatAccFiltered(self, cmd):
        if (self.plot_done2):
            self.trajectory_lateral_acc_filtered = cmd
            self.plot_done2 = False

    def CallBackTrajRaw(self, cmd):
        if (self.plot_done3):
            self.trajectory_raw = cmd
            self.plot_done3 = False

    def CallBackTrajTimeResampled(self, cmd):
        if (self.plot_done4):
            self.trajectory_time_resamped = cmd
            self.plot_done4 = False

    def CallBackTrajFinal(self, cmd):
        if (self.plot_done5):
            self.trajectory_final = cmd
            self.plot_done5 = False
        
    def timerCallback(self, event):
        self.plotTrajectory()
        self.plot_done1 = True
        self.plot_done2 = True
        self.plot_done3 = True
        self.plot_done4 = True
        self.plot_done5 = True

    def CalcArcLength(self, traj):
        s_arr = []
        ds = 0.0
        s_sum = 0.0

        if len(traj.points) > 0:
            s_arr.append(s_sum)

        for i in range(1, len(traj.points)):
            p0 = traj.points[i-1]
            p1 = traj.points[i]
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            s_sum += ds
            s_arr.append(s_sum)
        return s_arr

    def ToVelList(self, traj):
        v_list = []
        for p in traj.points:
            v_list.append(p.twist.linear.x)
        return v_list

    def CalcAcceleration(self, traj):
        a_arr = []
        for i in range(1, len(traj.points) - 1):
            p0 = traj.points[i-1]
            p1 = traj.points[i]
            v0 = p0.twist.linear.x
            v1 = p1.twist.linear.x
            v = 0.5 * (v1 + v0)
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            dt = ds / max(abs(v), 0.001)
            a = (v1 - v0) / dt
            a_arr.append(a)
        if len(traj.points) > 0:
            a_arr.append(0)
            a_arr.append(0)
        return a_arr

    def CalcJerk(self, traj):
        j_arr = []
        for i in range(1, len(traj.points) - 2):
            p0 = traj.points[i-1]
            p1 = traj.points[i]
            p2 = traj.points[i+1]
            v0 = p0.twist.linear.x
            v1 = p1.twist.linear.x
            v2 = p2.twist.linear.x

            dx0 = p1.pose.position.x - p0.pose.position.x
            dy0 = p1.pose.position.y - p0.pose.position.y
            ds0 = np.sqrt(dx0**2 + dy0**2)

            dx1 = p2.pose.position.x - p1.pose.position.x
            dy1 = p2.pose.position.y - p1.pose.position.y
            ds1 = np.sqrt(dx1**2 + dy1**2)

            dt0 = ds0 / max(abs(0.5*(v1+v0)), 0.001)
            dt1 = ds1 / max(abs(0.5*(v2+v1)), 0.001)

            a0 = (v1 - v0) / max(dt0, 0.001)
            a1 = (v2 - v1) / max(dt1, 0.001)
            j = (a1 - a0) / max(dt1, 0.001)
            j_arr.append(j)
        if len(traj.points) > 0:
            j_arr.append(0)
            j_arr.append(0)
            j_arr.append(0)
        return j_arr

    def plotTrajectory(self):
        plt.clf()
        ax1 = plt.subplot(3,1,1)#row, col, index(<raw*col)
        x = self.CalcArcLength(self.trajectory_raw)
        y = self.ToVelList(self.trajectory_raw)
        if len(x) == len(y):
            ax1.plot(x, y, label="0: raw", marker="")

        x = self.CalcArcLength(self.trajectory_external_velocity_limitted)
        y = self.ToVelList(self.trajectory_external_velocity_limitted)
        if len(x) == len(y):
            ax1.plot(x, y, label="1: external_velocity_limitted", marker="")

        x = self.CalcArcLength(self.trajectory_lateral_acc_filtered)
        y = self.ToVelList(self.trajectory_lateral_acc_filtered)
        if len(x) == len(y):
            ax1.plot(x, y, label="2: lateral_acc_filtered", marker="*")

        x = self.CalcArcLength(self.trajectory_time_resamped)
        y = self.ToVelList(self.trajectory_time_resamped)
        if len(x) == len(y):
            ax1.plot(x, y, label="3: time_resamped", marker="*")

        x = self.CalcArcLength(self.trajectory_final)
        y = self.ToVelList(self.trajectory_final)
        if len(x) == len(y):
            ax1.plot(x, y, label="4: final", marker="*")
        ax1.set_title("trajectorys velocity")
        ax1.legend()
        ax1.set_xlim([0, 50])
        ax1.set_ylabel("vel [m/s]")

        ax2 = plt.subplot(3,1,2)
        x = self.CalcArcLength(self.trajectory_final)
        y = self.CalcAcceleration(self.trajectory_final)
        if len(x) == len(y):
            ax2.plot(x, y, label="final")
            ax2.set_xlim([0, 50])
            ax2.set_ylim([-5, 5])
        ax2.set_ylabel("acc [m/ss]")


        # ax2.legend()

        ax3 = plt.subplot(3,1,3)
        x = self.CalcArcLength(self.trajectory_final)
        y = self.CalcJerk(self.trajectory_final)
        if len(x) == len(y):
            ax3.plot(x, y, label="final")
            ax3.set_xlim([0, 50])
            ax3.set_ylim([-2, 2])
        ax3.set_xlabel("arclength [m]")
        ax3.set_ylabel("jerk [m/sss]")

        # ax3.legend()

        #plt.show()
        plt.pause(.01)


def main():
    rospy.init_node("trajectory_visualizer")
    TrajectoryVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()