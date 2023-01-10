# master-thesis-wrk
## 1. Introduction  
this 
## 2. Usage
This repository uses the `mir robot` as a simulation example.  
Install the relevant repository through the following URL：
```
https://github.com/match-ROS/match_mobile_robotics.git
```
### 2.1 edit `move_base.launch` file
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
