# RRT_algorithm
RRT algorithm 3D in ROS
  
## 1.Prerequisities
- **ubuntu** 16.04, **ROS** Kinetic.
## 2.Build on ROS
  Clone the repository to your catkin workspace and catkin_make. For example:
```
    cd ~/catkin_ws/src
    git clone https://github.com/HandsomeTong66/RRT_algorithm.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
## 3.Usage
  Execute command in terminal:
```
    roscore
```
  
 ctrl+shift+t,Open a new page terminal and execute the following commands:
```
    rviz
```
and
```
   rosrun path_planning env_node 
   rosrun path_planning rrt_node
```
  
    
    
   <div align=center>
  <img src="https://user-images.githubusercontent.com/54161710/82776957-324a8080-9e7f-11ea-8806-4245493d8a9f.png" width = "360" height = "320">
  </div>

## 4.Reference
[nalin1096
/
path_planning](https://github.com/nalin1096/path_planning).
