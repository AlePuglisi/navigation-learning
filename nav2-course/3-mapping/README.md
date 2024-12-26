# 3. Generate a Map with SLAM <br/> (Simultaneous Localization and Mapping) 

This lesson does not include written code, but gaining confidence with robot teleoperation and Mapping using turtlebot3.<br/>
Take a look at [my notes](https://github.com/AlePuglisi/navigation-learning/blob/main/nav2-course/3-mapping/Lesson3_Mapping.pdf) for a detailed explanation of the lesson!

### Packages used:
- [``turtlebot3_cartographer``](https://github.com/ROBOTIS-GIT/turtlebot3/tree/main/turtlebot3_cartographer): for mapping
- [``turtlebot3_gazebo``](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_gazebo): for Gazebo simulation
- [``turtlebot3_teleop``](https://github.com/ROBOTIS-GIT/turtlebot3/tree/main/turtlebot3_teleop): for robot teleoperation
- [``nav2_map_server``](https://github.com/ros-navigation/navigation2/tree/main/nav2_map_server): to save the map

## Description 
To achieve the goal of autonomous navigation, our first step is map reconstruction.<br/>
Without a map, navigation in principle is possible but it may take a long time to navigate in an unknown environment to reach the goal pose.<br/>
The mapping procedure relies on SLAM algorithms, in which the robot simultaneously localizes itself in the environment (relative to obstacles and walls), and generates the best map estimation based on the sensor information.<br/>

What we need to experiment with mapping is a reliable simulation environment (or a real robot), and a way to move the robot around. 

## Tutorial 

> [!IMPORTANT]
> If Not done already, remember to specify TURTLEBOT3_MODEL environment variable, as explained [here](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course#quick-export-before-turtlebot3-tutorials)

### 1. Launch simulation environment
<image width=300 height=250 src=https://github.com/user-attachments/assets/5721b386-5d3f-4796-8e00-e7a3e1720bf2>

```bash
# Terminal 1
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
This will launch a Gazebo world with the turtlebot3 robot in it (including sensors such as 2D Lidar)  

### 2. Run teleoperation node
<image src=https://github.com/user-attachments/assets/ed9ef382-46cb-44e9-91ce-53e12f4e5953>

```bash
# Terminal 2
ros2 run turtlebot3_teleop teleop_keyboard 
```

When this terminal is your selected window, you can move the robot around using your keyboard.<br/>
This is very important for mapping, you have to move around and gather information to reconstruct the map.<br/>
As can be seen in the figure above, the nodes involved during teleoperation are:
- ``teleop_keyboard``: convert keyboard command into velocity commands, published to ``/cmd_vel`` topic
- ``turtlebot3_diff_drive``: controller node, implementing differential drive controller that converts chassis command into wheel command, based on differential drive kinematic.

### 3. Launch Mapping feature
<image src=https://github.com/user-attachments/assets/0cedeb77-5f77-418b-92f1-6766b3d53783>

```bash
# Terminal 3
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

> [!NOTE]
> ``use_sim_time:=True`` argument is needed only when working in Gazebo simulation.

This launcher is used to process map creation, and opens Rviz2, providing a real-time visualization of how mapping is going. <br/> 

Now, the teleoperation terminal is used to move the robot around and reconstruct the map, discovering all free (white) and occupied (black) space from sensor data. <br/>
You don't need to discover 100% of the reachable map, but obtain a meaningful representation, avoiding false wall openings or non-existent obstacles.  

> [!CAUTION]
> During robot motion around, avoid:
> - Turning too fast  (may cumulate noise)
> - Hitting obstacles (may create oscillations, that move the plane of 2D Laser scan)
> Those behaviors affect the quality of the generated map.<br/>
> If map data are too noisy due to these factors, restart everything.

Keep all terminals open all the time, or your mapping process will be lost!

### 4. Save the Map

Once you finish exploring the whole environment, and you are satisfied with the reconstructed map:

```bash
# Terminal 4
ros2 run nav2_map_server map_saver_cli -f <relative_path/map_name> 
```

This command will create two files in ``<relative_path/>`` (with respect to the terminal path where you run ``map_saver_cli`` node):
<br/>

<image align=right width=200 height=200 src=https://github.com/user-attachments/assets/48f1a48b-a28f-44de-8d65-005a04ad6aa9>
  
- ``<map_name>.pgm``: map image that will be loaded for navigation. This is characterized by: <br/>
  - **White pixels** for free space <br/>
  - **Black pixels** for occupied space (obstacles, walls) <br/>
  - **Grey pixels for** unknown space (for example outside of boundary or inside big obstacles) <br/>
    
- ``<map_name>.yaml``: it contains meta-information about the map: <br/>
  - **image**: relative path to pgm file <br/>
  - **resolution**: meter/pixel map resolution <br/>
  - **origin**: coordinates of the bottom-left point in the map, depending on the location where we start 
      mapping <br/>
  - **negate**: 0 by default, 1 if occupied/free are inverted <br/>
  - **occupied_tresh**: when the probability of a pixel being occupied is above this threshold, it will be considered occupied <br/>
  - **free_tresh**: when the probability of a pixel being occupied is below this threshold, it will be considered free <br/> 

## Conclusion
Following this step, it is possible to reconstruct a 2D map of any environment.<br/>
As a practice, you can try to map another turtlebot3 environment, by launching 
```bash
# Terminal 4
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
It may take a while to load the first time, but then you can perform mapping in a more realistic environment. 
