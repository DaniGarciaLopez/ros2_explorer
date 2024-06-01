# ROS 2 Turtlebot 3 Map Explorer
## Description
In this repo we use Turtlebot 3 along with ROS 2 and Gazebo to explore an unknown csv environment, navigate through it and create a map. 

The map is created using SLAM with the package [Google Cartographer](https://github.com/cartographer-project/cartographer) and navigation is achieved with [Nav2](https://github.com/ros-planning/navigation2) package. We have developed two exploring algorithyms:

>**Wanderer Exploration** explores the map doing random turns when it detects an obstacle. It's a convenient way to explore small maps but time consuming for bigger ones.
  
>**Discoverer Exploration** prioritizes specific unknown hotspots of the map convoluting the occupancy grid. It's a better way to explore bigger maps in exchange of a higher computational cost.

### [Youtube Video](https://youtu.be/UNiCngwE_Zo)
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/UNiCngwE_Zo/maxresdefault.jpg)](https://youtu.be/UNiCngwE_Zo)

## Installation (tested on Ubuntu 22.04 - ROS 2 Humble)

[Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html)

Don't forget to install colcon:
```
sudo apt install python3-colcon-common-extensions
```
Install Gazebo:
```
sudo apt install gazebo
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
Compile packages and get dependencies:
```
cd ~/turtlebot3_ws/src
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

cd ~/turtlebot3_ws/
colcon build
```
Include the following lines in ~/.bashrc:
```
source /opt/ros/humble/local_setup.bash
source ~/turtlebot3_ws/install/local_setup.bash

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=~/turtlebot3_ws/src/ros2_explorer/explorer_gazebo/models
```
## How to run
Execute the launch file and pass the map name (Opens Gazebo simulation, Rviz, Cartographer, Nav2 and exploration servers):
```
ros2 launch explorer_bringup explorer.launch.py map_name:=map10
```
Execute manager node and select the desired exploring algorithm:
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
## Package structure
![image](https://github.com/DaniGarciaLopez/ros2_explorer/blob/main/explorer_bringup/data/explorer_graph.png)
![image](https://github.com/DaniGarciaLopez/ros2_explorer/blob/main/explorer_bringup/data/rosgraph.png)

