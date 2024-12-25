# 3. Generate a Map with SLAM <br/> (Simultaneous Localization and Mapping) 

This lesson does not include written code, but gaining confidence with robot teleoperation and Mapping using turtlebot3.<br/>
Take a look at [my notes](https://github.com/AlePuglisi/navigation-learning/blob/main/nav2-course/3-mapping/Lesson3_Mapping.pdf) for a detailed explanation of the lesson!

### Packages used:
- [``turtlebot3_cartographer``](https://github.com/ROBOTIS-GIT/turtlebot3/tree/main/turtlebot3_cartographer): for mapping
- ``turtlebot3_gazebo``: for Gazebo simulation
- [``turtlebot3_teleop``](https://github.com/ROBOTIS-GIT/turtlebot3/tree/main/turtlebot3_teleop): for robot teleoperation
- [``nav2_map_server``](https://github.com/ros-navigation/navigation2/tree/main/nav2_map_server): to save the map

## Description 

## Tutorial 

> [!IMPORTANT]
> If Not done already, remember to specify TURTLEBOT3_MODEL environment variable, as explained [here](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course#quick-export-before-turtlebot3-tutorials)

### 1. Launch simulation environment
<image width=300 height=250 src=https://github.com/user-attachments/assets/5721b386-5d3f-4796-8e00-e7a3e1720bf2>

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
This will launch a Gazebo world with the turtlebot3 robot in it (including sensors such as 2D Lidar)  

### 2. Run teleoperation node

```bash
ros2 run turtlebot3_teleop teleop_keyboard 
```



## Generated Map
