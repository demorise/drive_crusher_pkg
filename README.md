# ROS2 Drive Crusher Package

## Installation
1. Open a terminal and navigate to the src folder of your ROS2 colcon workspace, then run the following commands:
```bash
git clone https://github.com/DoveConsulting/aaa.git
cd ..
colcon build --symlink-install
```

## Running
### Launch Robot in Rviz2
Open a terminal, navigate to the root of your colcon workspace and run the following commands:
```bash
source install/local_setup.bash
ros2 launch aaa_pkg robot_bringup.launch.py
``` 
### Run ROS2 Node
Open another terminal, navigate to the root of your colcon workspace and run the following commands:
```bash
source install/local_setup.bash
ros2 run aaa_pkg <ros2_node>
``` 
