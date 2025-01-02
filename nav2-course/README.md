# ROS2 Nav2 - with SLAM and Navigation
Ubuntu 22.04 | ROS 2 Humble | Nav2 | SLAM | Gazebo 

This Resource is based on the course on Nav2 on [Udemy](https://www.udemy.com/course/ros2-nav2-stack/?srsltid=AfmBOooiAWhc3jH4Gwttw345eHEBR6KJ7WLRfCRzbN5M8y_iSPS0GvtT&couponCode=KEEPLEARNING) by Edouard Renard. <br/>

:notebook: Download [here](https://github.com/AlePuglisi/navigation-learning/blob/main/nav2-course/ROS2_Nav2.pdf) my handwritten notes on this course! I like being repetitive and writing "useless" stuff, but obvious notes often helped, believe me!

>[!NOTE]
> This is not to advertise his course.<br/>
> Instead, I want to provide a tutorial for those approaching ROS2 Navigation with Nav2 for the first time or those wanting to recap it.<br/>
> This resource is also for the future me, as a simplified "documentation" to Nav2 usage, this is based on my understanding and what I think is most useful.<br/>

## Prerequisite knowledge (Advised):
- ROS 2 basics (Nodes, topics, etc)
- Python Programming 
- Linux CLI basics

## 1. Introduction 
This course has a learn-by-doing approach, and it is possible to follow it without a real robot, Everything here is done in Gazebo simulation.<br/>
It is easy to extend this tutorial to a real mobile robot.<br/>
Take a look at [my Notes](https://github.com/AlePuglisi/navigation-learning/blob/main/nav2-course/Lesson1_2_Intro_Setup.pdf) for further details.

### Learning steps: 
- Discover Nav2 stack with [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- Understand how the Nav2 stack works
- How to create a custom navigation world in Gazebo
- How to make your robot usable with Nav2
- Write code to interact with Nav2

### Benefits of using ROS 2 
- Provide the base layer for any robotics application
- "Avoid reinventing the wheel", work at a high level, without reimplementing stuff
- Amazing open-source community
- Plug and Play packages easy to use<br/>

In summary, ROS speeds up development time!<br/>
In particular, implementing a Navigation framework from scratch would be complex and time-consuming, but the Nav2 stack is here to help! 

### What is Nav2 stack, Why do we need it? 
<image width=400 height=200 src=https://github.com/user-attachments/assets/e038bcea-5b1e-4881-8704-82df6e5dcea1>
<image width=200 height=200 src=https://github.com/user-attachments/assets/6a1eb788-1991-400f-916f-83b0c2b1510b>

A Stack is a collection of ROS packages (a framework) to achieve a specific goal, in our case, Navigation: <br/>
Make the robot move from point A to point B (with a desired pose = position + orientation),<br/> 
in a safe way (avoiding static and dynamic obstacles).
<br/>

The navigation objective is achieved in 2 steps: 
  1) Create a representation of the environment map (SLAM)
  2) Make the robot navigate from A to B on that map

Both tasks are achieved with Nav2 functionalities and tools. <br/>
Also, Nav2 stacks expose many ways to interact with it from custom ROS 2 applications.

## 2. Setup and Installation 
To run the code of those lectures you need: 
- Ubuntu 22.04
- ROS 2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 
( Nav2 is implemented for previous ROS 2 distributions, but different bugs were present for Foxy)

Once ROS 2 and ``colcon`` are properly installed and the environment sourced, follow these steps to install the required packages to follow along: 

```bash
# Terminal 
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* 
```
### Quick export before turtlebot3 tutorials: 

<image align=left width=200 height=200 src=https://github.com/user-attachments/assets/24e0ac93-3c15-4f65-a3f6-4a826db6ba41>

The [turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robotic platform, comes in different models.<br/>
In this course, we use the **Waffle** model in Gazebo Simulation. <br/>
To communicate to the environment the model in use, let's export a variable.<br/>
As usual, to avoid doing it anytime you open a new terminal, add this export to your bashrc: 
```bash
# Terminal 
gedit ~/.bashrc
```
Add the line ``export TURTLEBOT3_MODEL=waffle`` at the end of the file, save, and close. 

### SOLVING ISSUES: 
1) **GAZEBO Simulation**:<br/>
    If you encounter problems when running Gazebo for simulation, try the following command in the terminal:
    ```bash
    # Terminal 
    . /usr/share/gazebo/setup.sh
    ```
    Launch Gazebo again, if now the problem is fixed, add that line to your bashrc. <br/>
    
    Another possible solution is to source the Gazebo setup.bash:
     ```bash
     # Terminal 
     source /usr/share/gazebo/setup.bash
     ```
     Again, if this solves the issue, add that line to your bashrc.<br/>
     
2) **NAVIGATION**:<br/>
   This fix is needed for [Section 4](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/4-navigate) on Navigation with Nav2.<br/>
   If launching ``turtlebot3_navigation2 navigation2.launch.py <args>``, the map is not uploaded in Rviz2, follow this 2 steps:<br/>

   - **Change DDS** used for ROS 2 communication:<br/>
     By changing the DDS from Fast to Cyclone, synchronization issues will be fixed.<br/>
     First, install the new DDS: 
     ```bash
     # Terminal 
     sudo apt install ros-humble-rmw-cyclonedds-cpp
     ```
     Then, communicate to your system that you want to use this DDS by adding this line to your bashrc: <br/>
     `` export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ``<br/>
   
   - **Modify Nav2 Parameter file**:<br/>
     To use turtlebot3 for Navigation, you need to make a quick fix on the parameter file related to the robot model you are going to use.<br/>
     During this course, we are going to use the **Waffle** model, so you are going to modify ``waffle.yaml``.<br/>
     First, move to the proper location, and then modify (with superuser rights) the parameter file: 
     ```bash
     # Terminal 
     cd /opt/ros/humble/share/turtlebot3_navigation2/param
     sudo gedit waffle.yaml
     ```
     Go to ``robot_model_type: "differential"`` and change it into ``robot_model_type: "nav2_amcl::DifferentialMotionModel"``
    
   


## 3. [Mapping](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/3-mapping) with SLAM 

How do you generate the world map using SLAM? <br/>
This is the first step required to achieve autonomous navigation. The generated map will be fundamental to Navigating the environment. 

## 4. [Navigate](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/4-navigate) with Nav2

Once a map has been generated, how do you navigate autonomously on it with Nav2? <br/>
Give Nav2 Goal as an objective pose or multiple goals as waypoints. 

## 5. [Architecture](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/5-architecture), understand Nav2 Stack

What are the main components of Nav2 Stack? <br/>
Up to this lesson, you learn the stack with [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3).<br/>
Understanding the global architecture and each component (global and local planner, etc..) is fundamental to set up a custom robot navigating in a custom environment.<br/>
Interacting with the Nav2 stack from external custom nodes also requires a better understanding of the architecture behind it!

## 6. [Custom World](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/6-custom-world)

How to map and navigate in **your custom simulation environment**? 
In this lecture, you will learn how to create your own world with the [Gazebo Building Editor](https://classic.gazebosim.org/tutorials?tut=building_editor).
Also, creating the world is not enough, details on how to integrate this new world in a ROS 2 package will be given. <br/>
Finally, you will get some tips on how to post-process the map obtained with SLAM, to get a better map for navigation. <br/>

## 7. [Custom Robot](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/7-custom-robot)

This is the time to leave behind our friend turtlebot3 and understand how to make your Robot compatible with Nav2 Stack. <br/>
Unfortunately, this is a hard process, here you will see just a steps overview. <br/>
In particular, by looking at the Nav2 Architecture inputs/outputs we will focus on: <br>
- Needed TFs (map->odom, odom->base_link, base_link->base_scan)
- Odometry (map->odom): How to provide it
- Sensor Data: How to set up sensors
- Hardware Controller: How to convert velocity commands into motor commands
Then, you will see what packages you need to launch SLAM and Navigation with a custom Robot and where the important configuration parameters are.

## 8. [Nav2 Interaction API](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/8-nav2-interaction)

What if you want to interact with Nav2 Stack from an external script (like a custom ROS 2 Node)?<br/>
Up to now, you have used Rviz2 to set Initial Pose and send Nav2 Goal or Waypoints, but it is not practical for a programmatic interaction. <br/>
In the background, Nav2 works with the usual ROS Communication tools, such as publish/subscribe, actions, etc...<br/>
So, it is possible to interact with it from external code, and there is no need to write your code from scratch, a very useful Python API exists!<br/>
Using [nav2_simple_commander](https://docs.nav2.org/commander_api/index.html) API, you will programmatically interact with Nav2 from a python script. 

## 9. [Conclusion]
