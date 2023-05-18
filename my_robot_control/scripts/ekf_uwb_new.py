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
class EKF_UWB_Publisher:
	def __init__(self):
		rospy.init_node('Node_EKF_UWB_Publisher', anonymous=True)
		#pub
		self.pose_uwb_r_pub         = rospy.Publisher("/debug/xEst_uwb_r",            Point, queue_size=50)    #50Hz
		self.pose_uwb_l_pub		   = rospy.Publisher("/debug/xEst_uwb_l",            Point, queue_size=50)

		self.pose_uwb_pub  = rospy.Publisher("/debug/pose_uwb",            Point, queue_size=50) # publish x,y,yaw (UWB)
		# self.xDR                    = rospy.Publisher("/debug/xDR",            Point, queue_size=50)    #50Hz

		#sub
		rospy.Subscriber('/agv/uwb/tag1', Tag, self.tag_uwb_l_callback)
		rospy.Subscriber('/agv/uwb/tag2', Tag, self.tag_uwb_r_callback)
		rospy.Subscriber('/dwm1001/position', Point, self.yaw_uwb_callback)
		rospy.Subscriber('/vel_pub', Twist, self.vel_callback)
		
		self.vel ={'v':0.0,'w':0.0}
		self.vel_uwb_r = {'v_r': 0.0,'w_r': 0.0}
		self.vel_uwb_l = {'v_l': 0.0,'w_l': 0.0}

		# w_r = self.vel['w']
		# v_r = self.vel['v'] + w_r*uwb_distance/2
		# self.vel_uwb_r = {'v_r': v_r,'w_r': w_r}
        
		# w_l = self.vel['w']
		# v_l = self.vel['v'] - w_l*uwb_distance/2
		# self.vel_uwb_l = {'v_l': v_l,'w_l': w_l}
		
		#for debug
		self.time_uwb = rospy.Time.now()
		self.time_yaw = rospy.Time.now()
		self.time_vel = rospy.Time.now()
		self.last_time  = rospy.Time.now()
		

		#for predict EKF
        
		self.time_ekf_update = rospy.Time.now()
		self.PEst_uwb_r = np.eye(3)*10 
		self.xEst_uwb_r = np.zeros((3, 1))

		self.PEst_uwb_l = np.eye(3)*10 
		self.xEst_uwb_l = np.zeros((3, 1))

		#toạ độ ban đầu
		self.ready_r = False
		self.ready_l = False
		
		self.uwb_distance = 0.75
        
		# self.pose     = {'x': ,'y': ,'th': }
		# self.pose_uwb_r = {'x': 0.,'y': 0.83,'th': 0.0}############ can do thuc te
		# self.pose_uwb_l = {'x': 0.52,'y': 1.59,'th': 0.0}
		self.pose_uwb_r = {'x': 1.66,'y': 1.41,'th': 0.0}############ can do thuc te
		self.pose_uwb_l = {'x': 1.66,'y': 2.16,'th': 0.0}############

		# self.pose_uwb_r = {'x': 3.2,'y': 1.41,'th': 0.0}############ can do thuc te
		# self.pose_uwb_l = {'x': 3.2,'y': 2.16,'th': 0.0}############

		# self.posexDR = {'x':1.42, 'y': 1.19, 'th': 0.0}
		# self.pose     = {'x':1.42, 'y':1.19,'th': 0.0}
        
		# self.posexDR  = {'x': ,'y': ,'th': }

		self.xEst_uwb_r[0,0] = self.pose_uwb_r['x'] 
		self.xEst_uwb_r[1,0] = self.pose_uwb_r['y']
		self.xEst_uwb_r[2,0] = self.pose_uwb_r['th']

		self.xEst_uwb_l[0,0] = self.pose_uwb_l['x']
		self.xEst_uwb_l[1,0] = self.pose_uwb_l['y']
		self.xEst_uwb_l[2,0] = self.pose_uwb_l['th']


		# self.x_uwb = 0
		# self.y_uwb = 0
		# self.yaw_uwb = 0
		self.yaw_uwb = 0.0
		
		self.R_IMU          = 0.1
		# self.R_UWB          = np.diag([0.31, 0.31]) ** 2
		self.R_UWB      = np.diag([0.31, 0.31, 0.01]) ** 2
		# self.R_UWB      = np.diag([0.8, 0.8, 0.01]) ** 2
		self.R_AMCL         = np.diag([0.01, 0.01, 0.01]) ** 2
		 
		self.Q = np.diag([  0.01, 0.01, 0.02]) ** 2 # OK predict state covariance
		self.Q1 = np.diag([0.4, 0.5])** 2    #v,w
		self.checkUWB_R = False   #kiem tra lan nhan dau tien
		self.checkUWB_L = False

	def update(self):
		if self.ready_r == True and self.ready_l == True:
			pass
			self.pub_pose_uwb(self.xEst_uwb_l, self.xEst_uwb_r)
			self.pub_pose_uwb_r(self.xEst_uwb_r)
			self.pub_pose_uwb_l(self.xEst_uwb_l)
			self.ready_l = False
			self.ready_r = False
			# self.pub_xDR_debug(self.posexDR)
			#rospy.loginfo("da nhan duoc du lieu tu ca hai uwb")
		# elif self.ready_r == True and self.ready_l == False:
		# 	pass
		# 	self.pub_pose_uwb_r(self.xEst_uwb_r)
		# elif self.ready_r == False and self.ready_l == True:
		# 	pass
		# 	self.pub_pose_uwb_l(self.xEst_uwb_l)
		# else:
		# 	pass
			#rospy.loginfo("khong co du lieu tu ca hai uwb")
			

	def pub_pose_uwb_r(self,xEst_uwb_r):
		pose_uwb_r_msg = Point()
		pose_uwb_r_msg.x = self.xEst_uwb_r[0,0]
		pose_uwb_r_msg.y = self.xEst_uwb_r[1,0]
		pose_uwb_r_msg.z = self.xEst_uwb_r[2,0]
		self.pose_uwb_r_pub.publish(pose_uwb_r_msg)

	def pub_pose_uwb_l(self,xEst_uwb_l):
		pose_uwb_l_msg = Point()
		pose_uwb_l_msg.x = self.xEst_uwb_l[0,0]
		pose_uwb_l_msg.y = self.xEst_uwb_l[1,0]
		pose_uwb_l_msg.z = self.xEst_uwb_l[2,0]
		self.pose_uwb_l_pub.publish(pose_uwb_l_msg)

	def pub_pose_uwb(self, xEst_uwb_l, xEst_uwb_r):
		pose_uwb_msg = Point()
		pose_uwb_msg.x = (self.xEst_uwb_l[0,0] + self.xEst_uwb_r[0,0])/2
		pose_uwb_msg.y = (self.xEst_uwb_l[1,0] + self.xEst_uwb_r[1,0])/2
		pose_uwb_msg.z = math.atan2(-(self.xEst_uwb_l[0,0]-self.xEst_uwb_r[0,0]),self.xEst_uwb_l[1,0]-self.xEst_uwb_r[1,0]) #atan2
		self.pose_uwb_pub.publish(pose_uwb_msg)
	
	def vel_callback(self,vel_msg):
		self.vel['v'] = vel_msg.linear.x    # v robot (m/s)
		self.vel['w'] = vel_msg.angular.z   # w robot (rad/s)
	
	# def pub_xDR_debug(self,pose):
	# 	current_time = rospy.Time.now()
	# 	dt = (current_time - self.last_time).to_sec()   #deta_time
	# 	self.last_time = current_time                   #update time

	# 	pose['x']   += self.vel['v']*dt*cos(pose['th'] + 0.5*self.vel['w']*dt)   # += v*TIME_SAMPE*cos(theta + 0.5*w*TIME_SAMPE);
	# 	pose['y']   += self.vel['v']*dt*sin(pose['th'] + 0.5*self.vel['w']*dt)   # += v*TIME_SAMPE*sin(theta + 0.5*w*TIME_SAMPE);
	# 	pose['th']  += self.vel['w']*dt                                        # += w*TIME_SAMPE;

	# 	pose['th'] = math.atan2(math.sin(pose['th']),math.cos(pose['th'])) # squash the orientation to between (-pi,pi)

	# 	pose_ = Point()
	# 	pose_.x = pose['x']
	# 	pose_.y = pose['y']
	# 	pose_.z = pose['th']
	# 	self.xDR.publish(pose_)

	def yaw_uwb_callback(self, yaw_uwb_msg):
		pass
		self.yaw_uwb = yaw_uwb_msg.z
	
	def tag_uwb_r_callback(self,uwb_r_msg):
		pass
		current_time = rospy.Time.now()
		Ts = (current_time - self.time_ekf_update).to_sec()
		self.time_ekf_update = current_time

		w_r = self.vel['w']
		v_r = self.vel['v'] + w_r*self.uwb_distance/2
		self.vel_uwb_r = {'v_r': v_r,'w_r': w_r}
		# u_r = np.array([v_r], [w_r])
		u_r = np.array([[self.vel_uwb_r['v_r']], [self.vel_uwb_r['w_r']]])
		rospy.loginfo("v_r = " + str(u_r[0,0]) + ", " + "w_r = " + str(u_r[1,0]))
		self.Q = np.diag([  0.01, 0.01, 0.02]) ** 2
		self.R_UWB      = np.diag([0.31, 0.31, 0.001]) ** 2
		# self.R_UWB      = np.diag([0.8, 0.8, 0.01]) ** 2
		if self.vel_uwb_r['v_r'] == 0 and self.vel_uwb_r['w_r'] == 0:
			pass
			self.Q = np.diag([  0.01, 0.01, np.deg2rad(0.01)]) ** 2 # OK predict state covariance
			self.R_UWB     = np.diag([0.31, 0.31, 10e4]) ** 2
			# self.R_UWB      = np.diag([0.31, 0.31, 0.001]) ** 2
		else:
			pass
			self.R_UWB      = np.diag([0.31, 0.31, 0.001]) ** 2
			# self.R_UWB      = np.diag([0.8, 0.8, 0.01]) ** 2
			
		if self.checkUWB_R == True:
			z = np.array([[uwb_r_msg.x],[uwb_r_msg.y],[self.yaw_uwb]])
			self.xEst_uwb_r, self.PEst_uwb_r = self.ekf_estimation_uwb(self.xEst_uwb_r, self.PEst_uwb_r, z, u_r, Ts)
			self.pose_uwb_r['x'] = self.xEst_uwb_r[0,0]
			self.pose_uwb_r['y'] = self.xEst_uwb_r[1,0]
			self.pose_uwb_r['th'] = self.xEst_uwb_r[2,0]
		self.ready_r = True
		self.checkUWB_R = True
              
		
	def tag_uwb_l_callback(self,uwb_l_msg):
		pass
		current_time = rospy.Time.now()
		Ts = (current_time - self.time_ekf_update).to_sec()
		self.time_ekf_update = current_time

		w_l = self.vel['w']
		v_l = self.vel['v'] + w_l*self.uwb_distance/2
		self.vel_uwb_l = {'v_l': v_l,'w_l': w_l}
		# u_l = np.array([v_l], [w_l])
		u_l = np.array([[self.vel_uwb_l['v_l']], [self.vel_uwb_l['w_l']]])
		rospy.loginfo("v_l = " + str(u_l[0,0]) + ", " + "w_l = " + str(u_l[1,0]))
		self.Q = np.diag([  0.01, 0.01, 0.02]) ** 2
		self.R_UWB      = np.diag([0.31, 0.31, 0.001]) ** 2
		# self.R_UWB      = np.diag([0.8, 0.8, 0.01]) ** 2
		if self.vel_uwb_l['v_l'] == 0 and self.vel_uwb_l['w_l'] == 0:
			pass
			self.Q = np.diag([  0.01, 0.01, np.deg2rad(0.01)]) ** 2 # OK predict state covariance
			self.R_UWB     = np.diag([0.31, 0.31, 10e4]) ** 2
			# self.R_UWB      = np.diag([0.31, 0.31, 0.001]) ** 2
		else:
			pass
			self.R_UWB      = np.diag([0.31, 0.31, 0.001]) ** 2
			# self.R_UWB      = np.diag([0.8, 0.8, 0.01]) ** 2
			
		if self.checkUWB_L == True:
			z = np.array([[uwb_l_msg.x],[uwb_l_msg.y],[self.yaw_uwb]])
			self.xEst_uwb_l, self.PEst_uwb_l = self.ekf_estimation_uwb(self.xEst_uwb_l, self.PEst_uwb_l, z, u_l, Ts)
			self.pose_uwb_l['x'] = self.xEst_uwb_l[0,0]
			self.pose_uwb_l['y'] = self.xEst_uwb_l[1,0]
			self.pose_uwb_l['th'] = self.xEst_uwb_l[2,0]
		self.ready_l = True
		self.checkUWB_L = True


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
		# print("UWB: Kx= " + str(K[0,0]) + ", Ky=" + str(K[1,1]) + ", Kth=" + str(K[2,2]))
		return xEst_uwb, PEst_uwb
        

# -------------Main--------------------
def main():
    print(msg)
    ekf = EKF_UWB_Publisher()
    ekf.spin()

if __name__ == '__main__':
    main()