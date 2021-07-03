# ROS 2 Turtlebot 3 Map Explorer
## Description
In this repo we use Turtlebot 3 along with ROS 2 and Gazebo to explore an unknown csv environment, navigate through it and create a map. 

The map is created using SLAM with the package [Google Cartographer](https://github.com/cartographer-project/cartographer) and navigation is achieved with [Nav2](https://github.com/ros-planning/navigation2) package. We have developed two exploring algorithyms:

>**Wanderer Exploration** explores the map doing random turns when it detects an obstacle. It's a convenient way to explore small maps but time consuming for bigger ones.
  
>**Discoverer Exploration** prioritizes specific unknown hotspots of the map convoluting the occupancy grid. It's a better way to explore bigger maps in exchange of a higher computational cost.

### [Youtube Video](https://youtu.be/UNiCngwE_Zo)
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/UNiCngwE_Zo/maxresdefault.jpg)](https://youtu.be/UNiCngwE_Zo)

## Installation (tested on Ubuntu 20.04 - ROS 2 Foxy)

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
Install Python libraries:
```
sudo apt install python3-pip
pip3 install pandas
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
## How to run
Execute the launch file of the map you want to use (Opens Gazebo simulation, Rviz, Cartographer, Nav2 and exploration servers):
```
ros2 launch explorer_bringup map10.launch.py
```
Execute manager node and select exploring algorithm:
```
ros2 run explorer_bringup manager
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
Maps will be converted to Gazebo format in `/explorer_gazebo/models` folder. Create a new .world.xml file in `/explorer_gazebo/worlds` and modify the name of the map you want to use:
```
<include>
  <uri>model://map1</uri>
</include>
```
Create a new launch file in `/explorer_bringup/launch` folder and modify the parameter `map_name` according to the map you just created.
## Testing commands
Cartographer launch:
```
ros2 launch explorer_cartographer cartographer.launch.py use_sim_time:=True
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
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```
## Package structure
![image](https://github.com/DaniGarciaLopez/ros2_explorer/blob/main/explorer_bringup/data/explorer_graph.png)
![image](https://github.com/DaniGarciaLopez/ros2_explorer/blob/main/explorer_bringup/data/rosgraph.png)

