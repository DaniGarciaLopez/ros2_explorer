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
Clone turtlebot original repository to have additional utilities (not necessary):
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
## Add your own CSV Map
Add your own csv maps in this folder:
```
cd ~/turtlebot3_ws/src/ros2_explorer/explorer_gazebo/maps/
```
Run Python script:
```
cd ~/turtlebot3_ws/src/ros2_explorer/explorer_gazebo/
python3 gazebo-map-from-csv.py
```
Maps will be converted to Gazebo format in `models` folder. Modify the name of the map you want to use in `worlds/map.world.xml`:
```
<include>
  <uri>model://map1</uri>
</include>
```
## How to run
```
ros2 launch explorer_bringup demo.launch.py
```

