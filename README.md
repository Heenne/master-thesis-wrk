# master-thesis-wrk
## 1. Introduction  
this 
## 2. Usage
This repository uses the `mir robot` as a simulation example.  
Install the relevant repository through the following URL：
```
https://github.com/match-ROS/match_mobile_robotics.git
```
### 2.1 Add path planner plugin
Browse into `your_catkin_ws/src/match_mobile_robotics/mir/mir_navigation/launch` and open the file `move_base.launch`.  
Add this code :
```
<param name="base_global_planner" value="path_planner/SplinedRelaxedAStar" />
<param name="base_global_planner" value="PRM_planner/PRMPlannerROS" />
<param name="base_global_planner" value="RRTstar_planner/RRTstarPlannerROS" />
```
under the code line:
```
<node pkg="move_base" type="move_base" respawn="false" name="move_base_node" output="screen" clear_params="true">
```
### 2.2 Run robot
```
roslaunch data_analysis run_robot.launch
```
In this launch file `run_robot.launch` you can change the map and start pose of the robot.  
### 2.3 Record
```
rosbag record /move_base_node/PRMPlannerROS/plan -O new.bag
rosbag record /move_base_node/RRTstarPlannerROS/plan -O new.bag
rosbag record /move_base_node/VoronoiPlannerROS/plan -O new.bag
```
### 2.4 set goal
```
roslaunch data_analysis run_robot.launch
```
### 2.5 data analysis
```
rostopic echo -b new.bag -p /move_base_node/RRTstarPlannerROS/plan > plan.csv
rostopic echo -b new.bag -p /move_base_node/PRMPlannerROS/plan > plan.csv
rostopic echo -b new.bag -p /move_base_node/VoronoiPlannerROS/voronoi_path > plan.csv
rostopic echo -b new.bag -p /move_base_node/SplinedRelaxedAStar/plan > plan.csv
```
