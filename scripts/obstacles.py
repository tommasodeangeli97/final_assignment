#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math

##globals
regions_ = {
    'right': 10,
    'fright': 10,
    'front': 10,
    'fleft': 10,
    'left': 10,
}

linear =0.5
angular=0.0


pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def controll():  #Controlling the parameter to stop the nodes that are not wanted
	ver_=rospy.get_param("modalita")
	if ver_ != 3:
		rospy.loginfo("this modality is temporary blocked, please put 'modality 3' in the instructions interface")
		controll()
	else:
		rospy.loginfo("thanks to have chosen that modality, let's go!")
		return 
		

def watch():   #I subscribe from the nodes 
	
	sub_scan=rospy.Subscriber('/scan', LaserScan, ViewCallback)
	sub_vel=rospy.Subscriber('/cmd_vel', Twist, VelocityCallback)
		

def ViewCallback(msg):  ##here I take the minimum distance between the robot and the wall
	global regions_
	regions_ = {
		'right':  min(min(msg.ranges[0:143]), 10),
		'fright': min(min(msg.ranges[144:287]), 10),
		'front':  min(min(msg.ranges[288:431]), 10),
		'fleft':  min(min(msg.ranges[432:575]), 10),
		'left':   min(min(msg.ranges[576:719]), 10),
	}

def VelocityCallback(msg):  ##here I take the actual velocty written on the cmd_vel topic
	global angular, linear
	
	angular = msg.angular.z
	linear = msg.linear.x
		
def action():  ##here I confront the two information below and i forbid the rotation
	global regions_ , linear , angular
	vel=Twist()
	watch()
	
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		controll()		
		if (regions_['right']< 0.2 and angular < 0.0):
			vel.angular.z=0.0
			vel.linear.x =linear
			
			rospy.loginfo("I can't turn this way")
			pub.publish(vel)
			continue
		if (regions_['right']<0.2 and angular>0.0):
			vel.angular.z = angular
			vel.linear.x = linear
			
			#rospy.loginfo("it's ok ")
			pub.publish(vel)
			continue
		if (regions_['fright']< 0.1 and angular <0.0):
			vel.angular.z=0.0
			vel.linear.x = linear
			 
			rospy.loginfo("I can't turn this way")
			pub.publish(vel)
			continue
		if (regions_['fright']<0.1 and angular>0.0):
			vel.angular.z = angular
			vel.linear.x = linear
			
			#rospy.loginfo("it's ok")
			pub.publish(vel)
			continue
		if (regions_['front']< 0.2 and linear > 0.0):
			vel.linear.x=0.0
			vel.angular.z = angular
			
			rospy.loginfo("I can't go this way")
			pub.publish(vel)
			continue
		if (regions_['fleft']< 0.1 and angular > 0.0):
			vel.angular.z=0.0
			vel.linear.x = linear
			
			rospy.loginfo("I can't turn this way")
			pub.publish(vel)
			continue
		if (regions_['fleft']<0.1 and angular<0.0):
			vel.angular.z = angular
			vel.linear.x = linear
			
			#rospy.loginfo("it's ok")
			pub.publish(vel)
			continue
		if (regions_['left']< 0.2 and angular > 0.0):
			vel.angular.z=0.0
			vel.linear.x = linear
			
			rospy.loginfo("I can't turn this way")
			pub.publish(vel)
			continue
		if (regions_['right']<0.2 and angular<0.0):
			vel.angular.z = angular
			vel.linear.x = linear
			
			#rospy.loginfo("it's ok")
			pub.publish(vel)
			continue
		elif (regions_['right']>0.2 and regions_['fright']>0.2 and regions_['front']>0.2 and regions_['fleft']>0.2 and regions_['left']>0.2):
			vel.angular.z = angular
			vel.linear.x = linear
			
			rospy.loginfo("it's ok")
			pub.publish(vel)
			continue
	
	rate.sleep()			
	
	
if __name__ == '__main__':
	
	try:
		rospy.init_node('obstacles', anonymous=True)
		controll()
		action()
		
	except rospy.ROSInterruptException:
		pass
		
		
