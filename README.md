# final_project
### After cloning the directory to your machine, run the following commands in this order:

### roslaunch fetch_gazebo playground.launch
### roslaunch fetch_moveit_config move_group.launch
### rosrun final_project fit_moveit.py

### you should see the end-effector move to a given location
### to give a different location, edit the fit_moveit.py script by changing the line pose = Pose(Point(0.047, 0.545, 1.822),
                          Quaternion(-0.374, -0.701, 0.173, 0.635))
