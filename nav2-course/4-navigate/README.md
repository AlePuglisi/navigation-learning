# 4. Make a Robot Navigate with Nav2

<image src=https://github.com/user-attachments/assets/87b642b8-839a-4980-ae9f-e57ddc037831>

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
This will launch the navigation functionality of the Nav2 stack, with Rviz properly configured for visualization. <br/>
``map:=`` argument is used to specify the relative path to the map file, saved preciously during SLAM. 

From now on, we will use Rviz GUI to interact with Nav2 Stack, to set the Initial Pose Estimate, Navigation Goal, or even multiple Goal Poses at once (waypoints). 

### 3. Select 2D Pose Estimate

<image width=420 height=300 src=https://github.com/user-attachments/assets/553f9909-a4f4-4bb3-b9b5-7f84340e5e8b>
<image src=https://github.com/user-attachments/assets/a7a055a8-ab66-4b66-93e8-ab7452e40b2b>

Before any navigation task, we need to initialize the Robot pose (position + orientation).<br/>
Until we do it, we cannot start any navigation request, and we have a Global Status Error on the Fixed Frame! <br/>

After clicking in Rviz on ``2D Pose Estimate`` as in the image above, go to the map with the cursor and position the cursor on the current (x,y) robot position
(in our case it will coincide with the map frame, due to how we have constructed the map). <br/>
Now, click on this (x,y) Robot position on the map, hold it, and orient the green arrow that appears in the correct robot orientation (in our case, towards the map x-axis). <br/>

If you define the correct initial pose, you will see the global/local cost maps and the Lidar Data correctly overlapping with the background map image. 

### 4.a. Send a Nav2 Goal 

<image  width=420 height=300 src=https://github.com/user-attachments/assets/ef5c208f-c889-4ce3-be01-c17f77ba06ff>

Once the Initial Pose has been correctly sent, we can start autonomous Navigation. <br/>

In Rviz, click on ``Nav2 Goal`` as in the image above, then position the cursor on the desired Goal position in the map. <br/>
Finally, click and orient the green arrow that appears with the desired orientation.<br/>

- If the pose selected is feasible, the robot will start to move autonomously towards it and send **REACHED** feedback when the task is completed.<br/>
  (We can see the feedback both on the terminal and in the bottom left "Navigation 2" section in Rviz). <br/>
- If the pose is unfeasible, a Recovery Behavior will start, and the navigation task will be **ABORTED**. 

### 4.b. Send Multiple Nav2 Goals (Waypoint Follower)

<image width=420 height=300 src=https://github.com/user-attachments/assets/74b317a1-b761-4fb3-901d-ee65a72ceff3>

When we want to send multiple Navigation Goals, we can do it by using multiple times the ``Nav2 Goal`` functionality, but it is not very efficient...<br/>
There exist a functionality just for this task, the ``Waypoint/Nav Through Poses Mode``, in the bottom left in Rviz, as in the image above. <br/>

After selecting it, proceed as you did for Sending a Nav2 Goal, but this time you can define multiple goals in sequence, as "waypoint i" (i=1,2,3,..).<br/>

Once all the desired waypoints have been defined in order, click on ``Start Waypoint Following``. <br/>
This functionality will start a waypoint navigation, stopping in all the poses with the given order. <br/>

If after defining the waypoints we select instead ``Start Nav Through Poses``, the Robot will try to define a path through all these waypoints, without stopping. <br/>
This functionality is useful for fast navigation, but it is not very stable, use it carefully!


### 5. Test Dynamic Obstacle Avoidance

An important feature of the Navigation stack is the dynamic obstacle avoidance.<br/>
Up to now, we have used a static map, with walls and obstacles already known from the map generated previously.<br/>
This map is used by the global planner to generate the global path, updated at low frequency.<br/> 
The local planner instead updates the trajectory at high frequency, according to the sensor data around the robot. <br/>

We will see how those planners and cost maps work in the [next lesson](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/5-architecture), this initial description is useful to understand this dynamic avoidance feature. <br/>

If during navigation unexpected obstacles appear on the path, thanks to sensor data, the map gets updated and as soon as the global planner replans the path, it will update it considering the new obstacles.<br/>

When the obstacle is not only unexpected but also moving with a fast dynamic, thanks to the higher local planner frequency update, it is possible to avoid it! <br/>

Here we will test how the path is replanned when an unexpected static obstacle appears in the map.<br/>
This can occur for example when we have the map of a building, but it is incorrect due to furniture added after the mapping generation.<br/>

<image src=https://github.com/user-attachments/assets/9e864797-6948-49b1-9d78-681ffc998aa0>

To test it in Gazebo: 
1. Start simulation world 
  ```bash
  # Terminal 1
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  ```
2. Start the Navigation stack
  ```bash
  # Terminal 2
  ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<relative_path/map_name.yaml>
  ```
3. Set 2D Pose Estimate
4. Send a Nav2Goal,<br/>
  **Before any Robot movement, Pause the Gazebo simulation as soon as the path is planned**
5. In Gazebo, go to the ``Insert`` section, and add an obstacle in the path
6. Play the Gazebo simulation (Re-click on Pause button)

## Conclusion
In this tutorial, you learn how to navigate in a mapped environment.<br/>
For now, you have to select a 2D Pose Estimate and send a Goal Pose from Rviz2, in [Lesson8](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/8-nav2-interaction) you will learn how to do it automatically, by interacting with Nav2 from external code.<br/>
The waypoint follower tool can be automated too, sending multiple goals from an external file.<br/>

You have a first view of global/local planner, recovery behavior, and reference frames (TFs), a detailed explanation of these concepts will be given in [Lesson 5](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/5-architecture).<br/>
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

