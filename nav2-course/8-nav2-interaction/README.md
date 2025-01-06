# 8. Interact Programmatically With the Navigation Stack
...WIP...

<image height=250, width = 800, src=https://github.com/user-attachments/assets/21105b70-3012-448a-b219-b54b236be8e8>

This lesson includes writing Python code, to interact with Nav2 without using Rviz GUI.<br/> 
Again, you will use turtlebot3 packages for the Robot and Navigation stack.<br/>
Now, the navigation commands (initial pose, goal pose, waypoints), will be sent using [nav2_simple_commmander API](https://docs.nav2.org/commander_api/index.html). <br/>
Take a look at [my notes](https://github.com/AlePuglisi/navigation-learning/blob/main/nav2-course/8-nav2-interaction/Lesson8_CommanderAPI.pdf) for a detailed explanation of the lesson!

### Packages used:
- [``turtlebot3_gazebo``](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_gazebo): for Gazebo Simulation
- [``turtlebot3_navigation2``](https://github.com/ROBOTIS-GIT/turtlebot3/tree/main/turtlebot3_navigation2) for Robot Navigation
- [``nav2_simple_commander``](https://github.com/ros-navigation/navigation2/tree/main/nav2_simple_commander) Python API to Nav2
  
## Description 

## How does Nav2 work behind the curtains? 

## Tutorial and script

### 0. Install the API
First, install the package to use the API in your Python code:

```bash
# Terminal 0
sudo apt install ros-<distro>-nav2-simple-commander 
```

### Write your custom Python code to interact with Nav2

For simplicity, here we will write a simple Python script to interact with Nav2, using nav2_simple_commander.<br/>
to fully integrate this in your ROS2 project, this functionality can be easily integrated into a ROS2 Node written in Python. <br/>

Open a terminal and move to the directory where you want to create the Python script

```bash
# Terminal 0
# Create the file
touch nav2_test.py
# Make it an executable
chmod +x nav2_test.py
```

Refer to [``nav2_test.py``](https://github.com/AlePuglisi/navigation-learning/blob/main/nav2-course/8-nav2-interaction/nav2_test.py), you can copy and paste it.<br/>

> [!NOTE]
> A big part of the code has been commented out, this is a previous implementation of some computations.<br/> 
> The same computations have been rewritten as functions, for reusability. <br/>
> I keep that unused code because it may help to understand the workflow. <br/>

### Test the Code
### 1. Launch Simulation Environment
<image width=300 height=250 src=https://github.com/user-attachments/assets/5721b386-5d3f-4796-8e00-e7a3e1720bf2>

```bash
# Terminal 1
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
This will launch a Gazebo world with the turtlebot3 robot in it (including sensors such as 2D Lidar)  

### 2. Launch Navigation Tool 
```bash
# Terminal 2
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<relative_path/map_name.yaml>
```
This will launch the navigation functionality of the Nav2 stack, with Rviz properly configured for visualization. <br/>
``map:=`` argument is used to specify the relative path to the map file, saved preciously during SLAM. 

From now on, we will use Rviz GUI to interact with Nav2 Stack, to set the Initial Pose Estimate, Navigation Goal, or even multiple Goal Poses at once (waypoints). 

### 3. Run the Python code to send Nav commands 
In a new Terminal, move to the directory where you create the Python script. <br/>

I manage the selection of the navigation task (initialization, single goal, waypoints) using command line arguments. <br/>
Every time you want to send a specific navigation task to Nav2, rerun the script with the appropriate argument: <br/>

- #### Initialize the Pose:
```bash
# Terminal 3
python3 nav2_test.py initialize 
```
- #### Send Single Nav2 Goal:
```bash
# Terminal 3
python3 nav2_test.py goal 
```
- #### Send Waypoints:
```bash
# Terminal 3
python3 nav2_test.py waypoints 
```

## Conclusion 

If you want to make this script even more general, use the arguments (accessed by ``sys.argv[]``) to define the Goal pose or the waypoints, instead of hardcoding it.<br/>
