#!/usr/bin/env python

import sys
import rospy
import roslaunch


def change():  #change the parameter to switch between the trhee modalities
	x=int(input("modality: "))
	rospy.set_param('modalita', x)
	command=sys.path("$(find) final_assignment/launch/start_project.launch")
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [command])
	tracking_launch.start()
	
	
if __name__ == '__main__':  #gives the possibilities
	try:
		rospy.init_node('instructions', anonymous=True)
		rospy.loginfo("Welcome to a simple robot simulation; we have three modalities implemented. Please insert: 'modality 1' to put the coordinates and let the robot autonomously reach them; 'modality 2' to guide the robot with the keyboard; 'modality 3' in order to get an assisted guide to avoid collisions.")
		change()
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass
