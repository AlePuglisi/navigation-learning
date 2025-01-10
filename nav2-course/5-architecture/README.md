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


A simple-to-use tool for parameter analysis and update can be found by launching ``rqt``, we will use it: 

> [!IMPORTANT]
> To visualize the parameters related to Nav2 with rqt, launch Robot simulation and Navigation. 

```bash
# Terminal
rqt
```

Once ``rqt`` window opens, go to:  > Plugins > Configuration > Dynamic Reconfigure



## Recovery Behaviors

## TFs and Important Frames

## Nav2 Architecture

## Conclusion 
Finally, you know what happens when the robot moves towards the Goal. <br/>
This knowledge is foundamental for the next lessons, in which you will explore how to customize the world in which a custom robot navigate. <br/>
For further details on all the basic concepts behind Nav2, you can look at the official [Documentation](https://docs.nav2.org/concepts/index.html).
