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
### How to Create Python Package
* First, go into ros workspace source directory. For exapmle: `cd ~/ros2_ws/src`
* To create a new package: `ros2 pkg create my_py_package --build-type ament_python --dependencies rclpy`
  * `ament_python` says in which language will be coding
  * `rclpy` says ros dependency will be used in python
* To compile this specific package, return the folder back `~/ros2_ws/`
  * type command `colcon build --packages-select my_py_package` 

### How to Create Python Package
* First, go into ros workspace source directory. For exapmle: `cd ~/ros2_ws/src`
* To create a new package: `ros2 pkg create my_cpp_package --build-type ament_cmake --dependencies rclcpp`
  * `ament_cpp` says in which language will be coding
  * `rclcpp` says ros dependency will be used in cpp
* To compile this specific package, return the folder back `~/ros2_ws/`
  * type command `colcon build --packages-select my_cpp_package`

## ROS Nodes
A package is an independent unit inside of an application. We will create nodes inside a package. Each node can be launced seperately.

![image](https://user-images.githubusercontent.com/35730346/131210576-77c0da8f-f45d-40f3-ac96-738e15776b79.png)

Nodes communicates with each other through topics, services, and parameters.

### Writing ROS Nodes in Python
* Go to the folder: `cd ~/ros2_ws/src/my_py_package/my_py_package`
* Generate the first node: `touch my_first_node.py` 
* Edit the file in VS Code:
  * A initial scheme for writing a node as below:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node #import Node class() from rclpy library

def main(args=None):
    #start communication with ROS2
    rclpy.init(args=args) #call __main__  args
    """
    .
    .

    write node codes between rclpy.init() and rclpy.shutdown()
    
    .
    .
    """
    node = Node("py_test") #The name of the node is not name of the file of node
    node.get_logger().info("Hello ROS2")
    
    rclpy.spin(node) #keeps the program running until exit

    #shutdown communication with ROS   
    rclpy.shutdown()
    
    #After shutdown, node will be destroyed, because the node will be out of scope

if __name__ == "__main__":
    main()
```

* Easy way to execute the node to test, executing python file directly. 
  * `chmod +x my_first_node.py`
  * `./my_first_node.py`
    
### How to install the node with Python
* `setup.cfg` file tells where you will install the file.
* update `setup.py` file `console_scripts`

```python
 .
 .
 description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #give a name(py_node) for executable, this will be in /install folder
            #use package name and node name, and call main function after semicolon(:)
            "py_node = my_py_package.my_first_node:main" 
        ],
 .
 .
```
* call `colcon build  --packages-select my_py_package` in `~/ros2_ws` to build again
* executable `py_node` will be created in the folder `~/ros2_ws/install/my_py_package/lib/my_py_package`
* before running `./py_node`, call `source ~/.bashrc`, otherwise there will be an error
