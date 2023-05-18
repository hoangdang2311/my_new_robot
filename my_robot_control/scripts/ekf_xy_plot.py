#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped, Point
import random
from itertools import count
import matplotlib.pyplot as plt
from math import sin, cos, pi
from matplotlib.pyplot import figure
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry
import numpy as np

class Info:
    x0 = 0
    y0 = 0  #xTrue

    x1 = 0
    y1 = 0  #xDR

    x2 = 0
    y2 = 0  #UWB

    x3 = 0
    y3 = 0  #xEst


plt.style.use('fivethirtyeight')
# t = []

x_xTrue = []
y_xTrue = []

x_xDR = []
y_xDR = []

x_uwb = []
y_uwb = []

x_xEst = []
y_xEst = []

def animate(i):
    plt.cla()
    
    plt.suptitle("xDR vs UWB vs EKF vs xTrue [XY Plot]")

    x_xTrue.append(Info.x0)
    y_xTrue.append(Info.y0)

    x_xDR.append(Info.x1)
    y_xDR.append(Info.y1)

    x_uwb.append(Info.x2)   
    y_uwb.append(Info.y2)

    x_xEst.append(Info.x3)
    y_xEst.append(Info.y3)
    
    # plt.scatter(x_xTrue, y_xTrue, marker='o', s=50, c='green', label='x_True')
    # plt.scatter(x_xDR, y_xDR, marker='o', s=50, c='black', label='x_DR')
    # plt.scatter(x_uwb, y_uwb, marker='o', s=50, c='blue', label='x_UWB')
    # plt.scatter(x_xEst, y_xEst, marker='o', s=50, c='red', label='x_UWB_EKF')
    plt.plot(x_xTrue, y_xTrue, linewidth=3, color='green', label="xTrue")
    # plt.plot(x_xDR, y_xDR, linewidth=3, color='black', label="xDR")
    # plt.plot(x_uwb, y_uwb, linewidth=3, color='blue', label="UWB")
    # plt.plot(x_xEst, y_xEst, linewidth=3, color='red', label="UWB_EKF")

    plt.xlabel('X(m)')
    plt.ylabel('Y(m)')
    # plt.xlim(-1,3)
    # plt.ylim(0,2)
    # plt.xticks([-1,0,1,2,3])
    # plt.yticks([0,1,2])
    plt.legend(loc="upper right")


def uwb_callback(uwb_data: Vector3):
    # rospy.loginfo("(" + str(uwb_data.x) + ", " + str(uwb_data.y) + ")")
    Info.x2 = float(uwb_data.x)
    Info.y2 = float(uwb_data.y)

def xDR_callback(xDR_data: Point):
    # rospy.loginfo("(" + str(xDR_data.x) + ", " + str(xDR_data.y) + ")")
    Info.x1 = float(xDR_data.x)
    Info.y1 = float(xDR_data.y)

def pose_uwb_ekf_callback(pose_uwb_data: Point):
    # rospy.loginfo("(" + str(pose_uwb_data.x) + ", " + str(pose_uwb_data.y) + ")")
    Info.x3 = float(pose_uwb_data.x)
    Info.y3 = float(pose_uwb_data.y)

def xTrue_callback(xTrue_data: Point):
    # rospy.loginfo("(" + str(xTrue_data.x) + ", " + str(xTrue_data.y) + ")")
    Info.x0 = float(xTrue_data.x)
    Info.y0 = float(xTrue_data.y)

if __name__ == '__main__':
    rospy.init_node("xy_subscriber")

    a = rospy.Subscriber("/dwm1001/position", Vector3, callback=uwb_callback)

    b = rospy.Subscriber("/debug/xDR", Point, callback=xDR_callback)

    c = rospy.Subscriber("/debug/pose_uwb_imu", Point, callback=pose_uwb_ekf_callback)

    d = rospy.Subscriber("/debug/xTrue", Point, callback=xTrue_callback)
    
    ani = FuncAnimation(plt.gcf(), animate, 200)

    plt.gcf().set_size_inches(12, 8)
    plt.tight_layout()
    plt.show()

    rospy.spin() 
