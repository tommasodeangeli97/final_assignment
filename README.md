# final_assignment
tird assignment: controll of a mobile robot

INSTALLATION
----
To install this package clone the package in your ROS workspace and print catkin_make

LAUNCH FILES:
----
startproject.launch
---
load a parameter by rosparam

launch all the nodes by <include> and the root to the others launch files

instructions.launch
---
launch the introduction node respawn="true" and launch-prefix="xterm-e"
  
printgoal.launch
---
launch the move_base_client node respawn="true" and launch-prefix="xterm-e"
  
modetwo.launch
---
launch the teleop_twist_keyboard node respawn="true" and launch-prefix="xterm-e"
  
modetrhee.launch
---
launch the obstacles node respawn="true" and launch-prefix="xterm-e"
  

PYTHON FILES:
----
introduction.py
---
start the node
  
gives the possibily to the user
  
takes the user input 
  
rospy.set_param() to change the parameter in the rosparameter server
  
move_base_client.py
---
import actionlib and move_base_msgs.msg
  
in the main, using controll(), I controll if the parameter have the right value to use that node
  
result=movebase_client()
  
starting the client and wait the server
  
takes the inputs by the user
  
gives the goal
  
wait until the timeout duration the respons by the server
  
communicate to the user the output and gives the possibility to change the goals
  
obstacles.py
---
import sensor_msgs.msg and geometry_msgs.msg
  
in the main controll() to watch the parameter
  
action()
  
in the action() I takes the callbacks to the velocity and the distance between the robot and the environment with watch()
  
start the publisher on cmd_vel, vel=Twist()
  
rospy.Rate(10) ten time in a second

confront the velocities and the distances
  
publish the velocities on cmd_vel and gives feedbacks to the user
  
rate.sleep
