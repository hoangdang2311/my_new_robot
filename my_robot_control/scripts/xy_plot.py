#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped, Point, PoseWithCovarianceStamped
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
    y0 = 0  #AMCL

    x1 = 0
    y1 = 0  #ODOM


plt.style.use('fivethirtyeight')

x_amcl = []
y_amcl = []

x_odom = []
y_odom = []

def animate(i):
    plt.cla()
    
    plt.suptitle("XY Plot")

    x_amcl.append(Info.x0)
    y_amcl.append(Info.y0)

    x_odom.append(Info.x1)
    y_odom.append(Info.y1)
    
    plt.plot(x_amcl, y_amcl, linewidth=3, color='green', label="AMCL")
    plt.plot(x_odom, y_odom, linewidth=3, color='red', label="ODOM")

    plt.xlabel('X(m)')
    plt.ylabel('Y(m)')
    plt.legend(loc="upper right")


def amcl_callback(amcl_data: PoseWithCovarianceStamped):
    # rospy.loginfo("(" + str(uwb_data.x) + ", " + str(uwb_data.y) + ")")
    Info.x0 = float(amcl_data.pose.pose.position.x)
    Info.y0 = float(amcl_data.pose.pose.position.y)

def odom_callback(odom_data: Odometry):
    # rospy.loginfo("(" + str(uwb_data.x) + ", " + str(uwb_data.y) + ")")
    Info.x1 = float(odom_data.pose.pose.position.x)
    Info.y1 = float(odom_data.pose.pose.position.y)


if __name__ == '__main__':
    rospy.init_node("xy_subscriber")

    a = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback=amcl_callback)

    b = rospy.Subscriber("/odom", Odometry, callback=odom_callback)
    
    ani = FuncAnimation(plt.gcf(), animate, 200)

    plt.gcf().set_size_inches(12, 8)
    plt.tight_layout()
    plt.show()

    rospy.spin() 
