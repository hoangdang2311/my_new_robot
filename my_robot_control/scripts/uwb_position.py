#!/usr/bin/python3

import rospy
import roslib
import tf

import PyKDL

import numpy as np
import math
from math import sin, cos, pi

from std_msgs.msg import String, Int8, Float32
from localizer_dwm1001.msg import Tag
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped


class UWB_Position:
    def __init__(self):
        rospy.init_node('Node_UWB_position', anonymous=True)
        self.dwm1001_position_pub = rospy.Publisher("/dwm1001/position",            Point, queue_size=50)

        rospy.Subscriber('/agv/uwb/tag1', Tag, self.uwb_l_callback)
        rospy.Subscriber('/agv/uwb/tag2', Tag, self.uwb_r_callback)

        self.dwm1001_pos = np.zeros((3, 1))
        self.uwb_l = np.zeros((3, 1))
        self.uwb_r = np.zeros((3, 1))
        self.check_uwb_l = False
        self.check_uwb_r = False

    def update(self):
        if self.check_uwb_l == True and self.check_uwb_r == True:
            pass
            self.pub_dwm1001_pos(self.dwm1001_pos)
            self.check_uwb_l = False
            self.check_uwb_r = False
    
    def pub_dwm1001_pos(self, dwm1001_pos):
        pos_dwm1001_msg = Point()
        pos_dwm1001_msg.x = (self.uwb_l[0,0] + self.uwb_r[0,0])/2
        pos_dwm1001_msg.y = (self.uwb_l[1,0] + self.uwb_r[1,0])/2
        pos_dwm1001_msg.z = math.atan2(-(self.uwb_l[0,0]-self.uwb_r[0,0]),self.uwb_l[1,0]-self.uwb_r[1,0])
        self.dwm1001_position_pub.publish(pos_dwm1001_msg)

    def uwb_l_callback(self, l_uwb_msg):
        self.uwb_l[0,0] = l_uwb_msg.x
        self.uwb_l[1,0] = l_uwb_msg.y
        self.check_uwb_l = True

    def uwb_r_callback(self, r_uwb_msg):
        self.uwb_r[0,0] = r_uwb_msg.x
        self.uwb_r[1,0] = r_uwb_msg.y
        self.check_uwb_r = True

    def spin(self):
        rospy.loginfo("[ROS] Start Node_UWB_Position")
        rate = rospy.Rate(50)   
        while not rospy.is_shutdown():
            self.update()            
            rate.sleep()

def main():
    pos = UWB_Position()
    pos.spin()


                
if __name__ == '__main__':
    main()