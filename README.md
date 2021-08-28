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
    
#### How to install the node with Python
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
##### - Two ways to execute the node
call `source ~/.bashrc`, otherwise there will be an error:
Executing by
1) calling the python file directly `./my_first_node.py` in the folder `cd ~/ros2_ws/src/my_py_package/my_py_package`

   Note: Make first python executable `chmod +x my_first_node.py`
   
2) calling `./py_node` in the folder `~/ros2_ws/install/my_py_package/lib/my_py_package`
3) calling ros2 run command as `ros2 run my_py_package py_node`

#### Improving The Node codes as OOP
The node file can be written by Object Oriented Program (OOP) approach as below. The function of the node is completely same, but in this version. The `node` object generated from a class named `MyNode` instead of generating from directly imported `Node` from rclpy.node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node #import Node class() from rclpy library

class MyNode(Node):

    def __init__(self):
        super().__init__("py_test") # call __init__ function from supervisor Node class()
        self.get_logger().info("Hello ROS2")

def main(args=None):
    #start communication with ROS2
    rclpy.init(args=args) #call __main__  args

    node = MyNode() #generate node object from the class of MyNode()
    
    rclpy.spin(node) #keeps the program running until exit

    #shutdown communication with ROS   
    rclpy.shutdown()
    
    #After shutdown, node will be destroyed, because the node will be out of scope

if __name__ == "__main__":
    main()
```
* compile again with `colcon build ..` as above
* then, run `ros2 run my_py_package py_node`

We can extend code by adding a timer and counter as below

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node #import Node class() from rclpy library

class MyNode(Node):

    def __init__(self):
        super().__init__("py_test") # call __init__ function from supervisor Node class()
        self.counter_ = 0
        self.get_logger().info("Hello ROS2")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello " + str(self.counter_))


def main(args=None):
    #Initilize ROS2 communication
    rclpy.init(args=args) #call __main__  args
    #Create Node
    node = MyNode()
    #Make the node spinning until press Ctrl + C
    rclpy.spin(node)
    #shutdown communication with ROS   
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
```
### Writing ROS Nodes in CPP
* Go to the folder: `cd ~/ros2_ws/src/my_cpp_package/src`
* Generate the first node: `touch my_first_node.cpp` 
* Edit the file in VS Code:
  * A initial scheme for writing a node as below:
* But, firstly, we need to link `*.hpp` files location, to do that:
  * Press Ctrl + Shift + P on VS Code, this active search bar. Then, write `C/C++: Edit Configurations (JSON)`
  * This generates a json file under the folder `.vscode`\
  * Add additional line `"/opt/ros/foxy/include"` for include list as below:
```json
  "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/foxy/include"
            ],
```

* After link the source of header files, update source codes of the node as below. This is a simple template for node generation: 

```cpp
#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv)
{
    // initilize communication with ROS2
    rclcpp::init(argc, argv); 
    // create a node by using shared pointer, no need to organize old/new pointers
    auto node = std::make_shared<rclcpp::Node>("cpp_test"); 
    // call a test function to print Hello on screen
    RCLCPP_INFO(node->get_logger(), "Hello Cpp Node");
    // Keeps the node is spinning
    rclcpp::spin(node);
    // Shutdown the communication with ROS2, when Ctrl + C pressed
    rclcpp::shutdown();
    return 0;
}
```

#### How to install the node after building
First, we need to change `CMakeLists.txt` file to build an executable file during `colcon build`
* `add_executable()` - generates executable file (first argument: node name, second argument: address of .cpp file)
* `ament_target_dependencies()` - links dependencies
* `install()` - installs executable which is generated above line. If not called, executable file will not be called without installation

```cpp
cmake_minimum_required(VERSION 3.5)
project(my_cpp_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}

)

ament_package()

```

##### - Two ways to execute the node
1) Go to the folder for executable `cd ~/ros2_ws/install/my_cpp_package/lib/my_cpp_package` and run `./cpp_node`
2) or first `source ~/.bashrc`, then run `ros2 run my_cpp_package cpp_node`

#### Improving The Node codes as OOP

```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node
{
public:
    MyNode()
        : Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");

        timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                         std::bind(&MyNode::timerCallback, this));
    }
private:
    void timerCallback()
    {
        counter_ ++;
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<MyNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

Please review the code above:
1) A class `MyNode()` has been added. This class has
 * a private `void` method named as `timerCallback()` triggers get_logger() function.
  * And 2 parameters which are only initilized, `timer_` and `counter_`
 * a public constructor with same name of class `MyNode`, and colon (:) initilizer to initilize `Node("cpp_test")` and also `counter_(0)`
2) In main method, an object named `node` has been instantiated from the class `MyNode()`  
