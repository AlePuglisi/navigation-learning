# 5. Understand the Nav2 Stack

...WIP...

<image width=640 heigth=400 src=https://github.com/user-attachments/assets/5748d36b-03d7-4340-8bb4-4f022f441e5d>

This lesson does not include writing code. <br/>
Refer to previous Lessons' commands and packages to explore the theoretical concepts explained here. <br/>

Being this Lesson purely theoretical, here I will provide just a summary, <br/>
take a look at [my notes](https://github.com/AlePuglisi/navigation-learning/blob/main/nav2-course/5-architecture/Lesson5_Nav2Architecture.pdf) for a detailed explanation of the lesson!

## Description 

So far we have been using Nav2 Stack with full confidence in its capabilities, but if you want to master its functionality, you need to know how it works. <br/>
In this tutorial, you won't see new commands and tools, but you will understand how the ones used until now achieve their goal. <br/>

In particular, I will give answers to the following questions: <br/>
- What are Global/Local Costmaps and Planners?
- Where are the parameters that configure Nav2?
- What is a Recovery Behavior?
- What are the important Frames for Nav2? <br/>

Once you understand these basic components, you are ready for the final question: 
- What is the overall Architecture of Nav2 Stack?

To experiment with the concept seen here, you need to follow the steps of [lesson 4, navigate](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/4-navigate).<br/>
You need a map, a simulation running on that map, and the Nav2 navigation tool. <br/>
For example: 

```bash
# Terminal 1
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```bash
# Terminal 2
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<relative_path/map_name.yaml>
```

## Global/Local Planner and Costmaps

<image width=400 heigth=400 src=https://github.com/user-attachments/assets/3d8193cb-e6b3-474e-a1cc-6ad017bd507b>
<image width=545 heigth=400 src=https://github.com/user-attachments/assets/85a2b197-f810-4c82-818b-5bb23be7450d>


## Parameters

I will focus on the most important configuration parameters, feel free to explore their functionality! <br/>
If you have some basic confidence with ROS, you know that parameters allow you to reconfigure the behavior of your project, in some cases even at runtime.<br/>
In an application like navigation, these can be very useful in defining safety limits to respect, control frequency, costmap characteristics, and so on. <br/> 
They are defined as usual in a ``.yaml`` configuration file, for example in the ``turtlebot3_navigation2`` package you can look here:

```bash
# Terminal
cd /opt/ros/humble/share/turtlebot3_navigation2/param
ls
```
As you can see, there is one configuration file per each turtlebot model. <br/>

#### Visualize Parameters with a GUI

<image width=600 heigth=300 src=https://github.com/user-attachments/assets/452d2bd7-7027-479a-8cdf-32d0869ccc6e>

A simple-to-use tool for parameter analysis and update can be found by launching ``rqt``, we will use it: 

> [!IMPORTANT]
> To visualize the parameters related to Nav2 with rqt, launch Robot simulation and Navigation.<br/>
> Remember also to Initialize the pose in Rviz to visualize the costmaps. 

```bash
# Terminal
rqt
```

Once ``rqt`` window opens, go to:  > Plugins > Configuration > Dynamic Reconfigure

#### Some Parameters Description: 
Here is a simple explanation of the role of some configuration parameters, look below for some comprehensive examples...<br/>

- **/global_costmap/global_costmap**: <br/>
   Under this, you find the live parameters for the global cost map configuration
  
    - **publish_frequency**: (Hz)<br/>
       Global Costmap update frequency 
      
    - **infation_layer.inflation_radius**: (m)<br/>
       Radius around obstacles at which we still consider the space not free.<br/>
       This ensures safe navigation, avoiding maneuvering too close to obstacles. <br/>
       A trade-off is needed, if too big even free space is hard to traverse, and if too small collision can occur.
      
    - **robot_radius**: (m) <br/> 
       Ostacle space (blue pixels), around mapped obstacles.<br/>
       It corresponds to the dimension of the robot as a circle of this radius. 

    - **resolution**: (m) <br/>
      Resolution of each pixel in the map
     ...
      
-  **/local_costmap/local_costmap**: <br/>
    Under this, you find the live parameters for the local cost map configuration (almost the same as in global_costmap)
    
    - **infation_layer.inflation_radius**: (m)<br/>
      Influence how the local planner path is computed. Define a safe "occupied" space around obstacles. <br/>
      As the global_costmap/inflation_layer.inflation_radius, this requires a trade-off for the tuning. <br/>
      If small: easy path computation, but high probability of collisions <br/>
      If big: less collision probability, but more complex path computation. 
    ...
      
- **/controller_server**: <br/>
   Under this, you find the live parameters for the Local Planner configuration
  
    - **max_vel**: (m/s) <br/>
      Upper limit on robot velocity (m/s)
      
    - **controller_frequency**: (Hz) <br/>
       Frequency at which the local planner sends velocity commands to the robot (usually ~ 10-100 Hz)
      
    - **goal_tolerance**: <br/>
       Admissible distance from goal location, to consider the goal reached. (requiring 0 pose error can lead to final instability) <br/>
       ...

#### Examples: 
For those parameters that may be hard to understand, let's visualize the effect of these: <br/>
(look at the pictures and remember the description above, the effect will be clearer. <br/>

- **/global_costmap/global_costmap/infation_layer.inflation_radius**

``inflation_radius=0.25``     |     ``inflation_radius=0.55``     |     ``inflation_radius=0.8``

<image width=325 heigth=250 src=https://github.com/user-attachments/assets/a73133da-8b2b-416e-bda0-401ca1626b4c>

<image width=325 heigth=250 src=https://github.com/user-attachments/assets/c67768af-c3dc-4bf5-a105-d1bf8d4406c1>
   
<image width=325 heigth=250 src=https://github.com/user-attachments/assets/07c02227-e578-4f21-8a78-74d646d4ceff>
   
- **/global_costmap/global_costmap/robot_radius**

``robot_radius=0.1``     |     ``robot_radius=0.22``

<image width=250 heigth=250 src=https://github.com/user-attachments/assets/7c15ed52-37c9-4560-a257-3c1caf806d84>

<image width=250 heigth=250 src=https://github.com/user-attachments/assets/9a2f1574-c1c9-4d1a-8740-8183972b70c2>

If you want to experiment with it, change the parameters numerical values on the rqt `` Dynamic Reconfigure``  Plugin, and look at the magic! <br/>
This knowledge can help you understand the other parameters!

## Recovery Behaviors

## TFs and Important Frames

## Nav2 Architecture

## Conclusion 
Finally, you know what happens when the robot moves towards the Goal. <br/>
This knowledge is foundamental for the next lessons, in which you will explore how to customize the world in which a custom robot navigate. <br/>
For further details on all the basic concepts behind Nav2, you can look at the official [Documentation](https://docs.nav2.org/concepts/index.html).
