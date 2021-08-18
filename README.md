# ROS_Usage
## Catkin

```cpp
workspace:~$ source /opt/ros/kinetic/setup.bash //source setup file
workspace:~$ mkdir catkin_ws //make an empty folder for catkin workspace
workspace:~$ cd catkin_ws
workspace:~/catkin_ws$ mkdir src //any Ros packages should be cloned into source folder to build 
workspace:~/catkin_ws$ cd src/ 
workspace:~/catkin_ws/src$ catkin_create_pkg give_a_package_name roscpp std_msgs //create a package name "give_a_package_name" depends on the "roscpp" and "std_msgs" libraries
workspace:~/catkin_ws/src$ cd ..
workspace:~/catkin_ws$ catkin_make //to build the catkin packages
```
