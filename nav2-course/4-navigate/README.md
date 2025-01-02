# 4. Make a Robot Navigate with Nav2

This lesson does not include writing code, but gaining confidence with robot autonomous Navigation using turtlebot3.<br/>
Take a look at [my notes](https://github.com/AlePuglisi/navigation-learning/blob/main/nav2-course/4-navigate/Lesson4_Navigate.pdf) for a detailed explanation of the lesson!

### Packages used:
- [``turtlebot3_gazebo``](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_gazebo): for Gazebo Simulation
- [``turtlebot3_navigation2``](https://github.com/ROBOTIS-GIT/turtlebot3/tree/main/turtlebot3_navigation2) for Robot Navigation

## Description 
If you follow the [mapping lesson](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/3-mapping), now you have an available world map, saved in your system.<br/>
You are now ready to Navigate in the mapped environment, going from the initial robot location to any feasible goal, avoiding both static and dynamic obstacles.<br/>
You will see how to send a goal or a set of goals (waypoints), what happens if the goal is unfeasible, and what if there are non-mapped (dynamic) obstacles.<br/>

## Tutorial 

> [!IMPORTANT]
> Using Nav2 in ROS 2 Humble, you may encounter this issue: <br/>
> When launching ``turtlebot3_navigation2  navigation2.launch.py``, Rviz2 is opened, but the map is not loaded, even if the arguments are given correctly.<br/>
> You can solve this issue by following the steps I explained [here](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course#solving-issues).

### 1. Launch simulation environment
<image width=300 height=250 src=https://github.com/user-attachments/assets/5721b386-5d3f-4796-8e00-e7a3e1720bf2>

```bash
# Terminal 1
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
This will launch a Gazebo world with the turtlebot3 robot in it (including sensors such as 2D Lidar)  

### 2. Launch Navigation feature

```bash
# Terminal 2
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<relative_path/map_name.yaml>
```

### 3. Select 2D Pose Estimate

### 4.a. Send a Nav2 Goal 

### 4.b. Send MUltiple Nav2 Goals (Waypoint Follower)

### 5. Test Dynamic Obstacle Avoidance

## Conclusion
In this tutorial, you learn how to navigate in a mapped environment.<br/>
For now, you have to select a 2D Pose Estimate and send a Goal Pose from Rviz2, in the next lectures you will learn how to do it automatically, by interacting with Nav2 from external nodes.<br/>
The waypoint follower tool can be automated too, sending multiple goals from an external file.<br/>

You have a first description of global/local planner, recovery behavior and reference frames (TFs), a detailed explanation of these concepts will be given in [Lesson 5](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/5-architecture).<br/>
You also see how, even in the presence of unexpected obstacles, Nav2 Stack can use sensor data to replan the path, avoiding collisions.<br/>

If you want to experiment with navigation in a more "realistic" environment, you can launch the house world (instead of the standard world) by: 
```bash
# Terminal 1
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
Remember that you need a map of this map, created following the same steps as [Mapping Lesson](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/3-mapping).<br/>
You can download ``my_house.pgm`` and ``my_house.yaml`` from [here](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/3-mapping/maps).

> [!NOTE]
> When navigating in a cluttered environment, you can see that navigation is less ideal than in the simple world of this Tutorial.<br/>
> For example when the robot has to traverse narrow spaces with a "U-shaped" trajectory, it may encounter issues and the goal may be aborted.<br/>
> You can try to solve unexpected behavior by redefining the Goal Pose.<br/>
> In the next lecture we will also see how to retune navigation parameters to optimize navigation!

