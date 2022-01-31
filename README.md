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
  

PYTHON FILES
----
