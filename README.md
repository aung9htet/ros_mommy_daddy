# ros_mommy_daddy

## To run the node

### Setting up the files (make sure it is in the directory for MiRo and not TB3)
1. Set the files into ```~/mdk/catkin_ws/src/```
2. Return to ```~/mdk/catkin_ws/``` and run ```catkin clean```
3. Run ```catkin build``` or ```catkin_make```
4. Run ```source devel/setup.bash```

### Running in diamond simulation
1. Run ```robot_switch miro```
2. Run ```roscore```
3. In another tab, go into ```~/mdk/catkin_ws/src/ros_mommy_daddy/ros_mommy_daddy_relationship/sim/``` and run ```rosrun gazebo_ros gazebo relationship.world```
4. In another tab, launch ```roslaunch ros_mommy_daddy_relationship relationship.launch```
