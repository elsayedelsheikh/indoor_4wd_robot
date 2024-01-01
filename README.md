# 4WD Robot Navigation 
This project is a ROS2 Navigation2 project for a Four Wheel Drive Robot. The robot is equipped with a Velodyne VLP-16 LiDAR, a raspberry pi 4B, and a micro-controller. The robot is able to navigate in an area using SLAM and localization. The robot is able to avoid obstacles and follow a path using the navigation system. The robot is able to be controlled using keyboard or/ joystick controller. The robot is able to build a map using SLAM with manual control.
## Launch System
To start the system, run the following command only:
```bash
ros2 launch bot_bringup bringup.launch.yaml
```
## Control the robot using keyboard
To control the robot using keyboard, run the following command:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
## Run individual subsystems
To start the LiDAR, run the following command, which will start the Velodyne driver and publish the point cloud data as well as the laser scan data:
```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
To start the robot state publisher which is required for the localization, run the following command:
```bash
ros2 launch bot_description bot_description.launch.py
```
To start the robot base controller which communicates with the micro-controller through USB Serial, run the following command:
```bash
ros2 launch bot_bringup bringup_comms.launch.yaml
```
To start the localization using AMCL algorithm using pre-saved map, run the following command:
```bash
ros2 launch bot_navigation localization.launch.py
```
To start the navigation in the area, run the following command:
```bash
ros2 launch bot_navigation navigation.launch.py
```
To build the map using SLAM, run the following command:
```bash
ros2 launch slam_toolbox online_async_launch.py
```
To visualize the robot in RViz2 and send navigation goals, run the following command:
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix bot_navigation)/share/bot_navigation/rviz/nav2_default_view.rviz
```
## To build the map using SLAM with manual control
To build the map using SLAM, run the following command:
```bash
ros2 launch slam_toolbox online_async_launch.py
```
To visualize the robot in RViz2 and send navigation goals, run the following command:
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix bot_navigation)/share/bot_navigation/rviz/nav2_default_view.rviz
```
To control the robot using keyboard, open a new terminal and run the following command:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
## System Configuration
### Navigation System
It starts with localization parameters with initial pose estimation, followed by used navigators, controllers, costmaps,  map server, planner, smoothers, recovery behaviors and waypoint follower. The parameters are self-explanatory.
In this project, Rotation Shim Controller is used to control the robot base where it is used to rotate the robot to the desired orientation. Then, RPP "Regulated Pure Pursuit" Controller is used to move the robot to the desired position to follow the path created by the planner. The planner used is the default planner in ROS2 Navigation2, which is the NavFnPlanner. The costmap used is the default costmap in ROS2 Navigation2, which is the Voxel Layer. The recovery behaviors used are the default recovery behaviors in ROS2 Navigation2, which are the Back Up, Clear Costmap, and Rotate Recovery. The waypoint follower used is the default waypoint follower in ROS2 Navigation2, which is the Follow Path.
To configure the navigation system, open the following file:
```bash
bot_navigation/params/nav2_params.yaml
```
### Waypoint Following
A set of waypoints are created to follow a path. The waypoints are created using the rviz2 tool using navigation2 plugin. The waypoints are saved in the following file:
```bash
bot_navigation/params/waypoints.yaml
```