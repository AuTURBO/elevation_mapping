# Elevation Mapping

# Run 

* Run elevation_mapping standalone:  

```bash
# If 'use_sim_time' is set to false, the node will be aborted with the following error: 
# terminate called after throwing an instance of 'std::runtime_error'
#  what():  can't subtract times with different time sources [2 != 1]
$ ros2 run elevation_mapping elevation_mapping_node --ros-args -p use_sim_time:=true  
```  

* Launch elevation_mapping with gazebo simulation:  

```bash
$ export TURTLEBOT3_MODEL=waffle
$ ros2 launch elevation_mapping_demos turtlesim3_waffle_demo.launch.py
```

```bash
# Open a new terminal.  
$ export TURTLEBOT3_MODEL=waffle
$ ros2 run turtlebot3_teleop teleop_keyboard
```  

* Tips for running the simulation:  

Sometimes, the simulation gets stuck at the termination. At this time, use the below command to kill the simulation.
```bash
$ killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient
```