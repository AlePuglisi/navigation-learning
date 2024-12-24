# ROS2 Nav2 - with SLAM and Navigation
Ubuntu 22.04 | ROS 2 Humble | Nav2 | SLAM | Gazebo 

This Resource is based on the course on Nav2 on [Udemy](https://www.udemy.com/course/ros2-nav2-stack/?srsltid=AfmBOooiAWhc3jH4Gwttw345eHEBR6KJ7WLRfCRzbN5M8y_iSPS0GvtT&couponCode=KEEPLEARNING) by Edouard Renard. <br/>

(..I'm currently taking this course..) <br/>
Notice that, this is not to advertise his course.<br>
Instead, I want to provide a tutorial for anyone approaching ROS2 Navigation with Nav2 for the first time, or for those that want to recap it.<br/>
This resource is also for the future me, as a simplified "documentation" to Nav2 usage, this is based on my understanding and what I think is most useful.<br/>

## Prerequisite knowledge (Advised):
- ROS 2 basics (Nodes, topics, etc)
- Python Programming 
- Linux CLI basics

## 1) Introduction 
This course has a learn-by-doing approach, and it is possible to follow it without a real robot, all is done with Gazebo simulation.<br/>
Anyway, it is easy to extend the knowledge to a real robot if available.<br/>

### Learning steps: 
- Discover Nav2 stack with [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- Understand how the Nav2 stack works
- How to create custom simulated world in Gazebo
- How to make your robot usable with Nav2
- Write code to interact with Nav2
  
### What is Nav2 stack, Why do we need it? 


## 2) Setup and Installation 

## 3) [Mapping](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/3-mapping) with SLAM 

How do you generate the world map using SLAM? <br/>
This is the first step required to achieve autonomous navigation. The generated map will be fundamental to Navigating the environment. 

## 4) [Navigate](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/4-navigate) with Nav2

Once a map has been generated, how do you navigate autonomously on it with Nav2? <br/>
Give Nav2 Goal as an objective pose or multiple goals as waypoints. 

## 5) [Architecture](https://github.com/AlePuglisi/navigation-learning/tree/main/nav2-course/5-architecture), understand Nav2 Stack

What are the main components of Nav2 Stack? <br/>
Up to this lesson, we learn the stack with [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3).<br/>
Understanding the global architecture and each component (global and local planner, etc..) is fundamental to set up a custom robot navigating in a custom environment.<br/>
Also, interacting with the Nav2 stack from external custom nodes require a better understanding of it!

## 6) ... WIP ...


