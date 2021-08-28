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

## Creating Package (Pkg)
## How to Create Python Package
* First, go into ros workspace source directory. For exapmle: `cd ~/ros2_ws/src`
* To create a new package: `ros2 pkg create my_py_package --build-type ament_python --dependencies rclpy`
  * `ament_python` says in which language will be coding
  * `rclpy` says ros dependency will be used in python
* To compile this specific package, return the folder back `~/ros2_ws/`
  * type command `colcon build --packages-select my_py_package` 

## How to Create Python Package
* First, go into ros workspace source directory. For exapmle: `cd ~/ros2_ws/src`
* To create a new package: `ros2 pkg create my_cpp_package --build-type ament_cmake --dependencies rclcpp`
  * `ament_cpp` says in which language will be coding
  * `rclcpp` says ros dependency will be used in cpp
* To compile this specific package, return the folder back `~/ros2_ws/`
  * type command `colcon build --packages-select my_cpp_package`
