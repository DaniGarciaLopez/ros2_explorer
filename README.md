# ros2_explorer
## Installation

[Install ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)

Create a ROS2 workspace:
```
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
```
Clone the repository:
```
git clone https://github.com/DaniGarciaLopez/ros2_explorer.git
```
Clone turtlebot original repository to have additional utilities (not necesary):
```
git clone foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
Compile packages:
```
cd ~/turtlebot3_ws/
colcon build
```
Include following lines in ~/.bashrc:
```
source /opt/ros/foxy/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/turtlebot3_ws
source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=~/turtlebot3_ws/src/ros2_explorer/explorer_gazebo/models
```
