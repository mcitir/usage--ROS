### Change Package name
Command | Comments
--------|----------
`ros2 run my_py_package py_node --ros-args --remap __node:=abc` | Passes a new argument via `--ros-args` to remap node name 
`ros2 run my_py_package py_node --ros-args -r __node:=abc` |
