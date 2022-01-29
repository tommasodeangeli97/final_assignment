#! /usr/bin/env python

from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the move_base action
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseResult

def controll():  #Controlling the parameter to stop the nodes that are not wanted
	ver_=rospy.get_param("modalita")
	if ver_ != 1:
		rospy.loginfo("this modality is temporary blocked, please put 'modality 1' in the instructions interface")
		controll()
	else:
		rospy.loginfo("thanks to have chosen that modality, let's go!")
		return 
		
def movebase_client():
    
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Waits until the action server has started up
    client.wait_for_server()
    
    # take the inputs
    x = float(input("x dest: "))
    y = float(input("y dest: "))

    # Creates a goal to send to the action server.
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id= "map"
    goal.target_pose.pose.orientation.w= 1.0
    goal.target_pose.pose.position.x=x
    goal.target_pose.pose.position.y=y
    goal.target_pose.header.stamp = rospy.Time.now()

    # Sends the goal to the action server.
    client.send_goal(goal)
    
    rospy.loginfo("I'm moving!")
    
    # Waits for the server to finish performing the action.
    wait = client.wait_for_result(timeout=rospy.Duration(45.0))

    # Gives the possibilities to the user and prints out the result of executing the action
    if( wait is False):
        rospy.loginfo("The position isn't reaced yet, do you want to change the destination?")
        req = str(input("insert 'yes' or 'no' : "))
        if (req == 'yes'):
            client.cancel_goal()
            
            movebase_client() 
        elif (req == 'no'):
            wait = client.wait_for_result(timeout=rospy.Duration(45.0))
            if( wait is False):
                rospy.loginfo("Unfortunatly the position can't be reached")
            else:
                return client.get_result()
    else:   
        return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node('move_base_client_py')
        while not rospy.is_shutdown():
        	controll()
        	result = movebase_client()
        	if result:
            		rospy.loginfo("I'm here! If you want going somewere else, insert the destination!")
            		controll()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
