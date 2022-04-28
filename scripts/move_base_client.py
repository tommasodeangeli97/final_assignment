#! /usr/bin/env python

"""
.. module:: move_base_client
   :platform: Unix
   :synopsis: Python module to interact with the move_base client, send goal and take the results of the oparation
   
.. moduleauthor:: Tommaso De Angeli

ROS node to interact with the move_base action, it's composed by two functions,
one is to control the modality of the application and the other is to interact with the move_base action

Actions:
  move_base to make the robot go to the indicated point
"""
from __future__ import print_function
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseResult

def controll():
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
	if ver_ != 1:
		rospy.loginfo("this modality is temporary blocked, please put 'modality 1' in the instructions interface")
		controll()
	else:
		rospy.loginfo("thanks to have chosen that modality, let's go!")
		return 
		
def movebase_client():
    """
    Function for developping the move_base client, it waits the user to insert the goal taken as coordinates x,y; it sends the goal to the action server and waits the result

    Args:
      none
  
    Returns:
      client.get_result(), function already existing in actionlib
  
    Function called:
      client.wait_for_server() to wait untill the connection with the server is stablished
  
    Function called:
      client.send_goal() to send the aquired goal to the actionlib
  
    Function called:
      client.wait_for_result() to wait untill the action is finished 
  
    Function called:
      client.cancel_goal() to delet the actual goal 
  
    Function called:
      client.get_result() to take the result of the action 
    """
    
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
    """
    Main function for colling the control of the parameter and inizialising a rospy node in order to give the possibility to the SimpleActionClient to communicate over ROS

    Function called:
      controll()
  
    Function called:
      movebase_client() called after the controll() function is done
    """
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
