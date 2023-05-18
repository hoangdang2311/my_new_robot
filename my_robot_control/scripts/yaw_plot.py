#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped, Point
import random
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from matplotlib.animation import FuncAnimation
from localizer_dwm1001.msg import Tag
from math import atan2, sin, cos

yaw_tmp = {'imu': 0.0, 'uwb': 0.0, 'ekf_uwb': 0.0, 'DR': 0.0}
pre_yaw = {'imu': 0.0, 'uwb': 0.0, 'ekf_uwb': 0.0, 'DR': 0.0}

class Info:
    yaw1 = 0
    yaw2 = 0
    yaw3 = 0
    yaw4 = 0
    time = 500
    k = time / 500
    index = 0

plt.style.use('fivethirtyeight')
t = []

yaw_imu = []
yaw_uwb = []
yaw_ekf_uwb = []
yaw_DR = []
h = 0

def animate(i):
    plt.cla()
    
    # plt.suptitle("Yaw graph")

    t.append(Info.index)

    Info.index += Info.k

    yaw_imu.append(Info.yaw1)
    yaw_uwb.append(Info.yaw2)
    yaw_ekf_uwb.append(Info.yaw3)
    yaw_DR.append(Info.yaw4)

    # plt.plot(t, h, linewidth=1, color='black')
    plt.plot(t, yaw_imu, linewidth=1, color='green', label="yaw")
    # plt.plot(t, yaw_uwb, linewidth=1, color='blue', label="yaw_uwb")
    # plt.plot(t, yaw_ekf_uwb, linewidth=1, color='red', label="yaw_ekf_uwb")
    # plt.plot(t, yaw_DR, linewidth=1, color='black', label="yaw_DR")

    plt.xlabel('Time(s)')
    plt.ylabel('Yaw(radian)')
    plt.legend(loc="upper right")

def imu_callback(imu_data: Vector3Stamped):
    global pre_yaw 
    global yaw_tmp
    # rospy.loginfo("yaw_imu = " + str(imu_data.vector.z))
    yaw_tmp['imu'] = float(imu_data.vector.z)
    delta = yaw_tmp['imu'] - pre_yaw['imu']
    delta = atan2(sin(delta), cos(delta))
    pre_yaw['imu'] = yaw_tmp['imu']
    Info.yaw1 += delta

def uwb_callback(uwb_data: Vector3):
    global pre_yaw 
    global yaw_tmp
    rospy.loginfo("yaw_uwb = " + str(uwb_data.z))
    yaw_tmp['uwb'] = float(uwb_data.z)
    delta = yaw_tmp['uwb'] - pre_yaw['uwb']
    delta = atan2(sin(delta), cos(delta))
    pre_yaw['uwb'] = yaw_tmp['uwb']
    Info.yaw2 += delta

def uwb_ekf_callback(uwb_ekf_data: Point):
    global pre_yaw 
    global yaw_tmp
    rospy.loginfo("yaw_uwb_ekf = " + str(uwb_ekf_data.z))
    yaw_tmp['ekf_uwb'] = float(uwb_ekf_data.z)
    delta = yaw_tmp['ekf_uwb'] - pre_yaw['ekf_uwb']
    delta = atan2(sin(delta), cos(delta))
    pre_yaw['ekf_uwb'] = yaw_tmp['ekf_uwb']
    Info.yaw3 += delta

def DR_callback(DR_data: Point):
    global pre_yaw 
    global yaw_tmp
    # rospy.loginfo("yaw_DR = " + str(DR_data.z))
    yaw_tmp['DR'] = float(DR_data.z)
    delta = yaw_tmp['DR'] - pre_yaw['DR']
    delta = atan2(sin(delta), cos(delta))
    pre_yaw['DR'] = yaw_tmp['DR']
    Info.yaw4 += delta

if __name__ == '__main__':
    rospy.init_node("velocity_subscriber")

    a = rospy.Subscriber("/imu/rpy/filtered", Vector3Stamped, callback=imu_callback)

    b = rospy.Subscriber("/dwm1001/position", Vector3, callback=uwb_callback)

    c = rospy.Subscriber("/debug/pose_uwb", Point, callback=uwb_ekf_callback)

    d = rospy.Subscriber("/debug/xDR", Point, callback=DR_callback)
    
    ani = FuncAnimation(plt.gcf(), animate, interval=Info.time)

    plt.gcf().set_size_inches(12, 8)
    plt.tight_layout()
    plt.show()

    rospy.spin() 
