#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped, Point

from urllib.request import urlopen, Request
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Bool
from math import atan2, sin, cos
import numpy as np

x = {'uwb': 0.0, 'true': 0.0, 'ekf_uwb': 0.0, 'DR': 0.0}
y = {'uwb': 0.0, 'true': 0.0, 'ekf_uwb': 0.0, 'DR': 0.0}
check_uwb = False
check_ekf_uwb = False
check_DR = False
check_true = False
shape = np.zeros((2,1))
x_DR = shape
x_True =shape
x_EKF =shape
x_UWB =shape
move_done =False
class LivePlotter(object):
    def __init__(self):
        self.fig = plt.figure(figsize=(15,12))
        self.fig.tight_layout()
        self.ax = self.fig.add_subplot(1, 1, 1)                                                                   
        self.time_interval = 200 #20
        self.x_data =  {'uwb': [], 'true': [], 'ekf_uwb': [], 'DR': []}
        self.y_data =  {'uwb': [], 'true': [], 'ekf_uwb': [], 'DR': []}

    def add_point_data(self, x_uwb, x_true, x_ekf_uwb, x_DR, y_uwb, y_true, y_ekf_uwb, y_DR):
        self.x_data['uwb'].append(x_uwb)
        self.x_data['true'].append(x_true)
        self.x_data['ekf_uwb'].append(x_ekf_uwb)
        self.x_data['DR'].append(x_DR)

        self.y_data['uwb'].append(y_uwb)
        self.y_data['true'].append(y_true)
        self.y_data['ekf_uwb'].append(y_ekf_uwb)
        self.y_data['DR'].append(y_DR)

    def animation(self, i):
        self.ax.clear()
        # self.ax.scatter(self.x_data['true'], self.y_data['true'], marker='o', s=50, edgecolors = 'g', facecolors='none', label="True")
        # self.ax.scatter(self.x_data['uwb'], self.y_data['uwb'], marker='o', s=50, color='blue', label="UWB")
        # self.ax.scatter(self.x_data['ekf_uwb'], self.y_data['ekf_uwb'], marker='o', s=50, color='red', label="EKF_UWB")
        # self.ax.scatter(self.x_data['DR'], self.y_data['DR'], marker='o', s=50, color='black', label="DR")
        self.ax.plot(self.x_data['true'], self.y_data['true'], linewidth=3, color='green', label="True")
        self.ax.plot(self.x_data['uwb'], self.y_data['uwb'], linewidth=3, color='blue', label="UWB")
        self.ax.plot(self.x_data['ekf_uwb'], self.y_data['ekf_uwb'], linewidth=3, color='red', label="EKF_UWB")
        self.ax.plot(self.x_data['DR'], self.y_data['DR'], linewidth=3, color='black', label="DR")
        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')
        self.ax.grid(visible=True, which='major', axis='both')
        self.ax.legend(loc="upper right")
 
        self.ax.set_xlim(0, 7.0)

        self.ax.set_ylim(0, 7.0)
        

    def show(self):
        self.ani = FuncAnimation(self.fig, self.animation, interval=self.time_interval)
        plt.show()

def uwb_callback(uwb_data: Vector3):
    global check_uwb
    global check_ekf_uwb
    global check_DR
    global check_true
    global x_UWB
    x['uwb'] = float(uwb_data.x)
    y['uwb'] = float(uwb_data.y)
    pose = np.array ([[uwb_data.x],[uwb_data.y]])
    x_UWB = np.hstack ((x_UWB,pose))
    check_uwb = True

def uwb_ekf_callback(uwb_ekf_data: Point):
    global check_uwb
    global check_ekf_uwb
    global check_DR
    global check_true
    global x_EKF
    x['ekf_uwb'] = float(uwb_ekf_data.x)
    y['ekf_uwb'] = float(uwb_ekf_data.y)
    check_ekf_uwb = True
    pose = np.array([[uwb_ekf_data.x],[uwb_ekf_data.y]])
    x_EKF = np.hstack((x_EKF,pose))

def DR_callback(DR_data: Point):
    global check_uwb
    global check_ekf_uwb
    global check_DR
    global check_true
    global x_DR
    x['DR'] = float(DR_data.x)
    y['DR'] = float(DR_data.y)
    check_DR = True
    pose = np.array([[DR_data.x],[DR_data.y]])
    x_DR = np.hstack((x_DR,pose))

def xTrue_callback(xTrue_data: Point):
    x['true'] = float(xTrue_data.x)
    y['true'] = float(xTrue_data.y)
    check_true = True
    if check_uwb == True and check_ekf_uwb == True and check_DR == True and check_true == True:
        plot.add_point_data(x['uwb'], x['true'], x['ekf_uwb'], x['DR'], y['uwb'], y['true'], y['ekf_uwb'], y['DR'])
    global x_True
    pose = np.array([[xTrue_data.x],[xTrue_data.y]])
    x_True = np.hstack((x_True, pose))

def move_callback(done_data):
    global move_done
    move_done = done_data.data
def calculate_rmse(a, b):
    N = min(a.shape[1], b.shape[1])
    x_rmse = np.sqrt(np.square(a[0, 1:N] - b[0, 1:N]).mean())
    y_rmse = np.sqrt(np.square(a[1, 1:N] - b[1, 1:N]).mean())
    return x_rmse,y_rmse


def main():
    rospy.init_node('node_xy_ekf_graph')
    # rospy.Subscriber("/imu/rpy/filtered", Vector3Stamped, callback=imu_callback)
    rospy.Subscriber("/dwm1001/position", Vector3, callback=uwb_callback)
    rospy.Subscriber("/debug/pose_uwb", Point, callback=uwb_ekf_callback)
    rospy.Subscriber("/debug/xDR", Point, callback=DR_callback)
    rospy.Subscriber("/debug/xTrue", Point, callback=xTrue_callback)
    rospy.Subscriber('/agv/move_done', Bool, callback = move_callback)
    # plot.add_point_data(yaw['imu'], yaw['uwb'], yaw['true'], yaw['ekf_uwb'], yaw['DR'])
    
    global move_done
    global x_DR,x_EKF,x_True,x_UWB
    while not rospy.is_shutdown():
        if move_done:
            move_done =False
            # print('RMSE IMU :' + str(calculate_rmse(th_IMU, x_True)))
            rospy.loginfo('RMSE UWB :' + str(calculate_rmse(x_UWB, x_True)))
            rospy.loginfo('RMSE DR  :' + str(calculate_rmse(x_DR, x_True)))
            rospy.loginfo('RMSE EKF :' + str(calculate_rmse(x_EKF, x_True)))
        plot.show()
            # rospy.spin()

if __name__ == '__main__':
    plot = LivePlotter()
    main()