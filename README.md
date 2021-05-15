# ROS2 Turtlebot3 Map Explorer
## Installation

[Install ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)

[Install Turtlebot3 on ROS2 Foxy](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

Don't forget to install colcon:
```
sudo apt install python3-colcon-common-extensions
```
Install Gazebo:
```
curl -sSL http://get.gazebosim.org | sh
```
Install packages:
```
sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-cartographer ros-foxy-cartographer-ros ros-foxy-navigation2 ros-foxy-nav2-bringup
sudo apt install ros-foxy-turtlebot3-msgs ros-foxy-dynamixel-sdk ros-foxy-hls-lfcd-lds-driver
```
Create a ROS2 workspace:
```
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
```
Clone the repository:
```
git clone https://github.com/DaniGarciaLopez/ros2_explorer.git
```
Clone turtlebot original repository to have additional utilities:
```
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
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
Launch basic simulation in gazebo loading map and robot in the initial position:
```
ros2 launch explorer_bringup demo.launch.py
```
Cartographer launch:
```
ros2 launch explorer_cartographer cartographer.launch.py use_sim_time:=True
```
Explore the map:
```
ros2 run explorer_wanderer wanderer
```
Navigation launch:
```
ros2 launch explorer_navigation2 nav.launch.py use_sim_time:=True

```
Move the robot manually:
```
ros2 run turtlebot3_teleop teleop_keyboard
```

Save a map:
```
ros2 run nav2_map_server map_saver_cli
```
Publish a goal:
```
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```
## Package structure
![image](https://github.com/DaniGarciaLopez/ros2_explorer/blob/main/explorer_bringup/data/explorer_graph.png)

