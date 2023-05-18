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

def animate(i):
    plt.cla()
    
    plt.suptitle("Odometry [XY Plot]")

    x_xTrue.append(Info.x0)
    y_xTrue.append(Info.y0)
    
    plt.plot(x_xTrue, y_xTrue, linewidth=3, color='green')

    plt.xlabel('X(m)')
    plt.ylabel('Y(m)')


def odom_callback(odom_data: Odometry):
    # rospy.loginfo("(" + str(uwb_data.x) + ", " + str(uwb_data.y) + ")")
    Info.x0 = float(odom_data.pose.pose.position.x)
    Info.y0 = float(odom_data.pose.pose.position.y)


if __name__ == '__main__':
    rospy.init_node("xy_odom_subscriber")

    a = rospy.Subscriber("/odom", Odometry, callback=odom_callback)
    
    ani = FuncAnimation(plt.gcf(), animate, 200)

    plt.gcf().set_size_inches(12, 8)
    plt.tight_layout()
    plt.show()

    rospy.spin() 
