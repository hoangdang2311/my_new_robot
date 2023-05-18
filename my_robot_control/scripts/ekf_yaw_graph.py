#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped, Point

from urllib.request import urlopen, Request
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from math import atan2, sin, cos

speed = {'left': 0.0, 'right': 0.0}
setpoint = {'left': 0.0, 'right': 0.0}
yaw = {'imu': 0.0, 'uwb': 0.0, 'true': 0.0, 'ekf_uwb': 0.0, 'DR': 0.0}
yaw_tmp = {'imu': 0.0, 'uwb': 0.0, 'true': 0.0, 'ekf_uwb': 0.0, 'DR': 0.0}
pre_yaw = {'imu': 0.0, 'uwb': 0.0, 'true': 0.0, 'ekf_uwb': 0.0, 'DR': 0.0}
class LivePlotter(object):
    def __init__(self):
        self.fig = plt.figure(figsize=(15,12))
        self.fig.tight_layout()
        self.ax = self.fig.add_subplot(1, 1, 1)                                                                   
        self.time_interval = 20 #20
        self.cnt = 0
        self.time = 0
        self.data =  { 'time': [], 'yaw_imu': [], 'yaw_uwb': [], 'yaw_true': [], 'yaw_ekf_uwb': [], 'yaw_DR': []}

    def add_point_data(self, yaw_imu, yaw_uwb, yaw_true, yaw_ekf_uwb, yaw_DR):
        self.cnt += 1
        self.time = self.cnt*self.time_interval/1000.0 #1000
        self.data['time'].append(self.time)
        self.data['yaw_imu'].append(yaw_imu)
        self.data['yaw_uwb'].append(yaw_uwb)
        self.data['yaw_true'].append(yaw_true)
        self.data['yaw_ekf_uwb'].append(yaw_ekf_uwb)
        self.data['yaw_DR'].append(yaw_DR)

    def animation(self, i):
        self.ax.clear()
        self.ax.plot(self.data['time'], self.data['yaw_true'], linewidth=1, color='green', label="yaw_xTrue")
        self.ax.plot(self.data['time'], self.data['yaw_imu'], linewidth=1, color='purple', label="yaw_imu")
        self.ax.plot(self.data['time'], self.data['yaw_uwb'], linewidth=1, color='blue', label="yaw_uwb")
        self.ax.plot(self.data['time'], self.data['yaw_ekf_uwb'], linewidth=1, color='red', label="yaw_ekf_uwb")
        self.ax.plot(self.data['time'], self.data['yaw_DR'], linewidth=1, color='black', label="yaw_DR")
        self.ax.set_xlabel('time (s)')
        self.ax.set_ylabel('yaw (rad)')
        self.ax.grid(visible=True, which='major', axis='both')
        self.ax.legend(loc="upper right")
        if self.time <= 10:
            self.ax.set_xlim(0, 10.0)
        else:
            self.ax.set_xlim(0.0, self.time + 3.0)

    def show(self):
        self.ani = FuncAnimation(self.fig, self.animation, interval=self.time_interval)
        plt.show()

def imu_callback(imu_data: Vector3Stamped):
    global pre_yaw 
    global yaw_tmp
    rospy.loginfo("yaw_imu = " + str(imu_data.vector.z))
    yaw_tmp['imu'] = float(imu_data.vector.z)
    delta = yaw_tmp['imu'] - pre_yaw['imu']
    delta = atan2(sin(delta), cos(delta))
    pre_yaw['imu'] = yaw_tmp['imu']
    yaw['imu'] += delta

def uwb_callback(uwb_data: Vector3):
    global pre_yaw 
    global yaw_tmp
    rospy.loginfo("yaw_uwb = " + str(uwb_data.z))
    yaw_tmp['uwb'] = float(uwb_data.z)
    delta = yaw_tmp['uwb'] - pre_yaw['uwb']
    delta = atan2(sin(delta), cos(delta))
    pre_yaw['uwb'] = yaw_tmp['uwb']
    yaw['uwb'] += delta

def uwb_ekf_callback(uwb_ekf_data: Point):
    global pre_yaw 
    global yaw_tmp
    rospy.loginfo("yaw_uwb_ekf = " + str(uwb_ekf_data.z))
    yaw_tmp['ekf_uwb'] = float(uwb_ekf_data.z)
    delta = yaw_tmp['ekf_uwb'] - pre_yaw['ekf_uwb']
    delta = atan2(sin(delta), cos(delta))
    pre_yaw['ekf_uwb'] = yaw_tmp['ekf_uwb']
    yaw['ekf_uwb'] += delta

def DR_callback(DR_data: Point):
    global pre_yaw 
    global yaw_tmp
    rospy.loginfo("yaw_DR = " + str(DR_data.z))
    yaw_tmp['DR'] = float(DR_data.z)
    delta = yaw_tmp['DR'] - pre_yaw['DR']
    delta = atan2(sin(delta), cos(delta))
    pre_yaw['DR'] = yaw_tmp['DR']
    yaw['DR'] += delta

def xTrue_callback(xTrue_data: Point):
    global pre_yaw 
    global yaw_tmp
    rospy.loginfo("yaw_xTrue = " + str(xTrue_data.z))
    yaw_tmp['true'] = float(xTrue_data.z)
    delta = yaw_tmp['true'] - pre_yaw['true']
    delta = atan2(sin(delta), cos(delta))
    pre_yaw['true'] = yaw_tmp['true']
    yaw['true'] += delta
    plot.add_point_data(yaw['imu'], yaw['uwb'], yaw['true'], yaw['ekf_uwb'], yaw['DR'])

def main():
    rospy.init_node('node_yaw_ekf_graph')
    rospy.Subscriber("/imu/rpy/filtered", Vector3Stamped, callback=imu_callback)
    rospy.Subscriber("/dwm1001/position", Vector3, callback=uwb_callback)
    rospy.Subscriber("/debug/pose_uwb", Point, callback=uwb_ekf_callback)
    rospy.Subscriber("/debug/xDR", Point, callback=DR_callback)
    rospy.Subscriber("/debug/xTrue", Point, callback=xTrue_callback)

    # plot.add_point_data(yaw['imu'], yaw['uwb'], yaw['true'], yaw['ekf_uwb'], yaw['DR'])
    plot.show()
    rospy.spin()

if __name__ == '__main__':
    plot = LivePlotter()
    main()