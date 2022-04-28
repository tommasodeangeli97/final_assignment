#!/usr/bin/env python

"""
.. module:: obstacles
   :platform: Unix
   :synopsis: Python module that scans the obstacles and control the velocity
   
.. moduleauthor:: Tommaso De Angeli

ROS node to avoid the collision with the obstacles, continously running

Subscribes to:
   /scan where the robot see the laser scanner
   
Subscribes to:
   /cmd_vel where is written the velocity of the robot
   
Publishes to:
   /cmd_vel to change the velocity of the robot
"""

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

"""
Global variables for the scanner, the linear velocity and the angular velocity
"""

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

"""
Publisher that publishes on /cmd_vel when the robot is too close to a wall for avoiding the collision
"""

def controll():  #Controlling the parameter to stop the nodes that are not wanted

	"""
	Function for controlling the parameter, if the parameter is setted on the required value the node will start, otherwise the function continously calls itself

	Args:
	  none
  
	Returns:
	  none
  
	Function called:
	  controll()
	"""
	ver_=rospy.get_param("modalita")
	if ver_ != 3:
		rospy.loginfo("this modality is temporary blocked, please put 'modality 3' in the instructions interface")
		controll()
	else:
		rospy.loginfo("thanks to have chosen that modality, let's go!")
		return 
		

def watch():   #I subscribe from the nodes 

	"""
	Function for reading the information given by the two topics 

	Subscribes to:
	  /cmd_vel for knowing the actual velocity of the robot
  
	Subcribes to:
	  /scan to read the information of distances given by the scanner on the robot
  
	Args:
	  none
  
	Returns:
	  none
	"""
	
	sub_scan=rospy.Subscriber('/scan', LaserScan, ViewCallback)
	sub_vel=rospy.Subscriber('/cmd_vel', Twist, VelocityCallback)
		

def ViewCallback(msg):  ##here I take the minimum distance between the robot and the wall

	"""
	Function callback for the scanner, gives the minimum distance value to the global variable *regions_*

	Args:
	  msg(LaserScan) given by the subrcriber sub_scan in the watch() function to control the robot distance
  
	Returns:
	  none
	"""
	global regions_
	regions_ = {
		'right':  min(min(msg.ranges[0:143]), 10),
		'fright': min(min(msg.ranges[144:287]), 10),
		'front':  min(min(msg.ranges[288:431]), 10),
		'fleft':  min(min(msg.ranges[432:575]), 10),
		'left':   min(min(msg.ranges[576:719]), 10),
	}

def VelocityCallback(msg):  ##here I take the actual velocty written on the cmd_vel topic

	"""
	Function callback for the velocity, gives the actual velocity value to globals variables angular and linear

	Args:
	  msg(Twist) given by the subscriber sub_vel in the watch() function to control the robot motion
  
	Returns:
	  none
	"""
	global angular, linear
	
	angular = msg.angular.z
	linear = msg.linear.x
		
def action():  ##here I confront the two information below and i forbid the rotation

	"""
	Function for countinously confronting the information stored in the global variables and for publishing on the /cmd_vel in order to avoid the collision with the walls. The function takes all the
	global variables and defines vel=Twist() to publish on the topic

	Publishes to:
	  /cmd_vel
  
	Args:
	  none
  
	Returns:
	  none
  
	Function called:
	  watch()
	"""
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

	"""
	Main function to inizialise the node and call the other functions

	Function called:
	  controll() to see if the modality is the required one
  
	Function called:
	  action() is launched when the controll() function is done
	"""
	
	try:
		rospy.init_node('obstacles', anonymous=True)
		controll()
		action()
		
	except rospy.ROSInterruptException:
		pass
		
		
