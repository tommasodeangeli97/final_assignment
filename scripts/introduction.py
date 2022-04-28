#!/usr/bin/env python

"""
.. module:: introduction
   :platform: Unix
   :synopsis: Python module, user interface for changing modality
   
.. moduleauthor:: Tommaso De Angeli

ROS node that set the parameter in the parameter server, continuosly running

The parameter is taken from the parameter server created here, in the launch file 
   :mod:'go_to_point' /root/my_ros_ws/src/final_assignment/launch/startproject.launch
"""

import sys
import rospy
import roslaunch


def change():  
	"""
	Function for taking the value of the parameter from the user and setting it in the parameter server 

	Args:
	  none

	Returns:
	  none
	"""
	x=int(input("modality: "))
	rospy.set_param('modalita', x)
		
	
if __name__ == '__main__':  
	"""
	Main function that continuosly inizializes the node and improves a user interface

	Function called:
	  change()
	"""
	try:
		rospy.init_node('instructions', anonymous=True)
		rospy.loginfo("Welcome to a simple robot simulation; we have three modalities implemented. Please insert: 'modality 1' to put the coordinates and let the robot autonomously reach them; 'modality 2' to guide the robot with the keyboard; 'modality 3' in order to get an assisted guide to avoid collisions.")
		change()
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass
