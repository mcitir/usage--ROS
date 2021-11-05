### Building ROS Workspace
Command | Comments
--------|----------
`sudo apt update` | update
`sudo apt install python3-colcon-common-extensions` | install colcon builder
`colcon build` | builds all packages
`colcon build --packages-select package_name` | builds only `package_name`
`colcon build --packages-select package_name --symlink-install` | creates a symbolic link to `*.py` file directly, instead of executable. It is **only** for **PYTHON**. Also, the python file should be executable, to do that, you should apply `chmod +x my_first_node.py` in the folder `~/ros2_ws/src/my_py_package/my_py_package/`. Otherwise, `--symlink-install` will give an error.


### Package
Command | Comments
--------|----------
`ros2 run node_name package_name` | runs a package of a node
`rosdep install -i --from-path-src --rosdistro foxy -y` | Resolve dependencies to check all dependencies whether installed 

#### Package (Exapmle) Installation Step:
1. `sudo apt install ros-foxy-navigation2`
2. `sudo apt install ros-foxy-nav2-bringup`
3. `sudo apt install ros-foxy-turtlebot3*`
4. `mkdir -p ~/ros_ws/src` (if not existed)
5. `cd ~/nav2_ws/src`
6. `git clone https://github.com/ros-planning/navigation2.git --branch foxy-devel`
7. `cd ~/nav2_ws
8. `rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy`
9. `colcon build --symlink-install` or
10. `colcon build --packages-select navigation2` to build only this package, not all workspace
11. Add the below links to `~/.bashrc` file

```bash
source ~/ros_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
source /usr/share/gazebo/setup.sh
```
12.  `cd ~/nav2_ws`
13.  `ros2 launch nav2_bringup tb3_simulation_launch.py`

### Node
Command | Comments
--------|----------
`ros2 node list` | lists active nodes
`ros2 node info /node_name` | shows info about node `node_name`


### Change Node name
Command | Comments
--------|----------
`ros2 run my_py_package py_node --ros-args --remap __node:=abc` | Passes a new argument via `--ros-args` to remap node name 
`ros2 run my_py_package py_node --ros-args -r __node:=abc` |

### Topics
Command | Comments
--------|----------
`ros2 topic list` | lists active topics
`ros2 topic echo /topic_name` | print topic_name message to screen
`ros2 topic info /topic_name` | gives information about`topic_name`: Number of Subscriber/Publisher and topic message type 

### Change Topic name
Command | Comments
--------|----------
`ros2 run my_py_package publisher_node --ros-args -r __node:=abc -r published_old_topic_name:= published_new_topic_name` | changes node and topic name, but be careful, subcribers can not subscribe after changing `published_old_topic_name`
`ros2 run my_py_package subcriber_node --ros-args -r published_old_topic_name:= published_new_topic_name` | subcribes directly to `published_new_topic_name`

### Publish a Topic from Terminal
Command | Comments
--------|----------
`ros2 topic pub -r 10 /robot_news example_interfaces/msg/String "{data: 'hello from terminal'}"` | 10:HZ, /robot_news:topic_name, example_in.../String:topic_message_type

### Install Turtlesim Package
Command | Comments
--------|----------
`sudo apt install ros-foxy-turtlesim` | installs turtlesim package if it is not installed

### 
