#!/usr/bin/python3

msg = """
    Author: Xin la xin 1 lan
    ------------------------------------
"""

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

uwb_distance = 0.76
class EKF_UWB_Position_Publisher:
	def __init__(self):
		rospy.init_node('Node_EKF_UWB_Position_Publisher', anonymous=True)
		#pub
		self.pose_uwb_pub  = rospy.Publisher("/debug/pose_uwb",            Point, queue_size=50) # publish x,y,yaw (UWB)

		#sub
		rospy.Subscriber('/dwm1001/position', Point, self.uwb_position_callback)
		rospy.Subscriber('/vel_pub', Twist, self.vel_callback)
		
		self.vel ={'v':0.0,'w':0.0}
		self.vel_uwb_r = {'v_r': 0.0,'w_r': 0.0}
		self.vel_uwb_l = {'v_l': 0.0,'w_l': 0.0}
		
		#for debug
		self.time_uwb = rospy.Time.now()
		self.time_yaw = rospy.Time.now()
		self.time_vel = rospy.Time.now()
		self.last_time  = rospy.Time.now()
		

		#for predict EKF
        
		self.time_ekf_update = rospy.Time.now()

		self.PEst_uwb = np.eye(3)*10 
		self.xEst_uwb = np.zeros((3, 1))

		#toạ độ ban đầu
		self.ready = False
		
		self.uwb_distance = 0.76
        
		# self.pose_uwb = {'x': 1.66,'y': 1.785,'th': 0.0}

		self.pose_uwb = {'x': 3.2,'y': 1.785,'th': 0.0}

		self.xEst_uwb[0,0] = self.pose_uwb['x']
		self.xEst_uwb[1,0] = self.pose_uwb['y']
		self.xEst_uwb[2,0] = self.pose_uwb['th']

		self.yaw_uwb = 0.0
		
		self.R_IMU          = 0.1
		# self.R_UWB          = np.diag([0.31, 0.31]) ** 2
		self.R_UWB      = np.diag([0.31, 0.31, 0.01]) ** 2
		# self.R_UWB      = np.diag([0.8, 0.8, 0.01]) ** 2
		self.R_AMCL         = np.diag([0.01, 0.01, 0.01]) ** 2
		 
		self.Q = np.diag([  0.01, 0.01, 0.02]) ** 2 # OK predict state covariance
		self.Q1 = np.diag([0.4, 0.5])** 2    #v,w
		self.checkUWB_R = False   #kiem tra lan nhau dau tien
		self.checkUWB_L = False
		self.checkUWB = False
		

	def update(self):
		if self.ready == True:
			pass
			self.pub_pose_uwb(self.pose_uwb)
		else:
			pass
			#rospy.loginfo("khong co du lieu tu ca hai uwb")

	def pub_pose_uwb(self, pose_uwb):
		pose_uwb_msg = Point()
		pose_uwb_msg.x = self.pose_uwb['x']
		pose_uwb_msg.y = self.pose_uwb['y']
		pose_uwb_msg.z = self.pose_uwb['th']
		self.pose_uwb_pub.publish(pose_uwb_msg)
	
	def vel_callback(self,vel_msg):
		self.vel['v'] = vel_msg.linear.x    # v robot (m/s)
		self.vel['w'] = vel_msg.angular.z   # w robot (rad/s)
	

	def uwb_position_callback(self,uwb_position_msg):
		pass
		current_time = rospy.Time.now()
		Ts = (current_time - self.time_ekf_update).to_sec()
		self.time_ekf_update = current_time

		u = np.array([[self.vel['v']], [self.vel['w']]])

		self.Q = np.diag([  0.01, 0.01, 0.02]) ** 2
		self.R_UWB      = np.diag([0.31, 0.31, 0.001]) ** 2
		# self.R_UWB      = np.diag([0.8, 0.8, 0.01]) ** 2
		if self.vel['v'] == 0  and self.vel['w'] == 0:
			pass
			self.Q = np.diag([  0.01, 0.01, np.deg2rad(0.01)]) ** 2 # OK predict state covariance
			self.R_UWB     = np.diag([0.31, 0.31, 0.01]) ** 2
			# self.R_UWB      = np.diag([0.31, 0.31, 0.001]) ** 2
		else:
			pass
			self.R_UWB      = np.diag([0.31, 0.31, 0.001]) ** 2
			# self.R_UWB      = np.diag([0.8, 0.8, 0.01]) ** 2
			
		if self.checkUWB == True:
			z = np.array([[uwb_position_msg.x],[uwb_position_msg.y],[uwb_position_msg.z]])
			self.xEst_uwb, self.PEst_uwb = self.ekf_estimation_uwb(self.xEst_uwb, self.PEst_uwb, z, u, Ts)
			self.pose_uwb['x'] = self.xEst_uwb[0,0]
			self.pose_uwb['y'] = self.xEst_uwb[1,0]
			self.pose_uwb['th'] = self.xEst_uwb[2,0]
		self.ready = True
		self.checkUWB = True


	def spin(self): #ok
		rospy.loginfo("[ROS] Start Node_EKF_UWB_Publisher")
		rate = rospy.Rate(50)   
		while not rospy.is_shutdown():
			self.update()            
			rate.sleep()
#-------------------------------------------------------------
#--------------------EKF--------------------------------------
	def robot_model_system(self,x_uwb, u_uwb,Ts):
        # Ref: (2.20) chapter (2.2.2)
		w_uwb = u_uwb[1,0]
		theta_uwb = x_uwb[2,0]
		# F = [3x3]
		F = np.array([[1.0, 0, 0],
		            [0, 1.0, 0],
		            [0, 0, 1.0]])  
		# B = [3x2]
		B = np.array([[Ts*math.cos(theta_uwb + 0.5*Ts*w_uwb),       0],
		            [Ts*math.sin(theta_uwb + 0.5*Ts*w_uwb),         0],
		            [0.0,                                   Ts]])
        # = [3x3][3x1] + [3x2][2x1] = [3x1]
        #x = F @ x + B @ u
		x_uwb = F.dot(x_uwb) + B.dot(u_uwb) 

		return x_uwb						


	def jacob_h_uwb(self):
        # Jacobian of Observation Model
		jH = np.array([
			[1, 0, 0],
			[0, 1, 0],
			[0, 0, 1]])
		return jH

	def jacob_f(self,x, u,Ts):
		v       = u[0, 0]     # v - robot
		w       = u[1, 0]     # w - robot
		theta   = x[2,0]      # yaw
		#jF = 3x3
		jF = np.array([
		    [1.0,   0.0,    -Ts * v * math.sin(theta + 0.5*Ts*w) ],
		    [0.0,   1.0,    Ts * v * math.cos(theta + 0.5*Ts*w)	 ],
            [0.0,   0.0,                                    1.0]])
		return jF 
 
	
	def ekf_estimation_uwb(self,xEst_uwb, PEst_uwb, z, u_uwb, Ts):
		# Predict
		xPred = self.robot_model_system(xEst_uwb, u_uwb, Ts)
		jF = self.jacob_f(xPred, u_uwb, Ts)
		PPred = jF.dot(PEst_uwb).dot(jF.T) + self.Q
		jH = self.jacob_h_uwb()
		zPred = jH.dot(xPred)
		S = jH.dot(PPred).dot(jH.T) + self.R_UWB
		K = PPred.dot(jH.T).dot(np.linalg.inv(S))
		xEst_uwb = xPred + K.dot((z - zPred)) #
		PEst_uwb = (np.eye(len(xEst_uwb)) - K.dot(jH)).dot(PPred)
		print("UWB: Kx= " + str(K[0,0]) + ", Ky=" + str(K[1,1]) + ", Kth=" + str(K[2,2]))
		return xEst_uwb, PEst_uwb
        

# -------------Main--------------------
def main():
    print(msg)
    ekf = EKF_UWB_Position_Publisher()
    ekf.spin()

if __name__ == '__main__':
    main()