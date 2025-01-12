# 8. Interact Programmatically With the Navigation Stack

<image height=250, width = 800, src=https://github.com/user-attachments/assets/21105b70-3012-448a-b219-b54b236be8e8>

This lesson includes writing Python code, to interact with Nav2 without using Rviz GUI.<br/> 
Again, you will use turtlebot3 packages for the Robot and Navigation stack.<br/>
Now, the navigation commands (initial pose, goal pose, waypoints), will be sent using [nav2_simple_commmander API](https://docs.nav2.org/commander_api/index.html). <br/>
Take a look at [my notes](https://github.com/AlePuglisi/navigation-learning/blob/main/nav2-course/8-nav2-interaction/Lesson8_CommanderAPI.pdf) for a detailed explanation of the lesson!

### Packages used:
- [``turtlebot3_gazebo``](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_gazebo): for Gazebo Simulation
- [``turtlebot3_navigation2``](https://github.com/ROBOTIS-GIT/turtlebot3/tree/main/turtlebot3_navigation2) for Robot Navigation
- [``nav2_simple_commander``](https://github.com/ros-navigation/navigation2/tree/main/nav2_simple_commander) Python API to Nav2

### Additional Library 
- ``tf_transformations``: used for Euler angle  to quaternion orientation conversion
  
## Description 

Up to now, we interact with Nav2 using Rviz GUI. <br/>
This is not convenient when you want to develop a complete project, maybe you want to send a goal from your own ROS 2 node. <br/>
If you consider a vision-based navigation project, once we identify the goal position with a computer vision algorithm, we send this goal automatically from our node. <br/>
To do so, we rely on a Python API to interact with Nav2, the nav2_simple_commander. <br/>

In this tutorial, you will understand the basics of nav2_simple_commander and write a simple Python script to initialize the Pose, send Nav2 Goals, and start the waypoints following. <br/>

But before coding, let's understand how the API interacts with the ROS2 communication Network.


## How does Nav2 works behind the curtains? 

Behind the scenes of Nav2, we find the usual ROS2 communication and interaction tools, topics, services, and actions.<br/>

Some examples are useful to understand what will happen next. <br/>
Launch the robot simulation and navigation stack, as explained before, and see with your eyes what are the topics, services, and actions available. <br/>

- **2D Pose Estimate**<br/>

The initialization of the Pose estimate is managed in Nav2 Stack from the message exchanged in the ``/initialpose`` topic.<br/>
Try to start the robot simulation and navigation tool as explained before, then open a new terminal and echo that topic. <br/>
As soon as you will set the 2D Pose Estimate from Rviz GUI; you will see it published on that topic! <br/>

```bash
# Terminal
ros2 topic info /initialpose
ros2 topic echo /initialpose
```

This topic subscription is used by the Nav2 stack to receive the 2D Pose Estimate. <br/>
Also, by investigating this topic info, we can see that it is not easy to fill this message. <br/>
This is a [``geometry_msgs/msg/PoseWithCovarianceStamped``](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseWithCovarianceStamped.html),<br/>
nav2_simple_commander helps us to initialize it with a simple Python method. 

- **Nav2 Goal**<br/>

This is a task that requires continuous feedback to manage step-by-step navigation toward the goal, you already know what you need, a ROS2 Action! <br/>
``navigate_to_pose`` is the action used for this.

```bash
# Terminal
ros2 action info /navigate_to_pose
```

- **Waypoint Follower**<br/>

``follow_waypoints`` is the action used for this.

```bash
# Terminal
ros2 action info /follow_waypoints
```

## Tutorial and script

### Install the API and additional Packages 
First, install the package to use the API in your Python code:

```bash
# Terminal 0
sudo apt install ros-<distro>-nav2-simple-commander 
```
Then, install an additional package and Python library used in this tutorial:<br/>

```bash
# Terminal 0
sudo apt install ros-<distro>-tf-transformations
sudo apt install python3-transforms3d 
```

This is needed to manage orientation conversion from Euler to quaternion.<br/>
Robot pose is defined in quaternion, being it easier to manage for computation.<br/>

> [!IMPORTANT]
> If when running the code you have errors related to tf_transformations import, upgrade it with pip
```bash
# Terminal 0
pip install --upgrade transforms3d
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

### Understanding nav2_test.py

For a detailed explanation of this Python API, take a look at the [documentation](https://docs.nav2.org/commander_api/index.html). <br/>
In this tutorial, I will briefly describe the methods used in this lesson. 

- ``BasicNavigator()``: <br/>
Object to initialize Nav2 API. This object contains all the methods to interact with Nav2. 

- ``BasicNavigator.setInitialPose(geometry_msgs.msg.PoseStamped)``: <br/>
Initialize the pose, even if Nav2 uses a ``PoseWithCovaranceStamped``, you can pass a simple ``PoseStamped``. <br/>
The proper definition of Covariance will be handled by the method. 

- ``BasicNavigator.waitUntilNav2Active()``: <br/>
Wait for Nav2 Stack to be ready after pose initialization. <br/>
Needed before starting any navigation task. 

- ``BasicNavigator.goToPose(geometry_msgs.msg.PoseStamped)``: <br/>
Start a navigation task toward the defined pose.<br/>

- ``BasicNavigator.followWaypoints(geometry_msgs.msg.PoseStamped[])``: <br/>
Start a "waypoint following" task, through the poses defined as a list of PoseStamped goals.<br/>

- ``BasicNavigator.isTaskComplete()``: <br/>
Return true when the task is completed, useful to define loops until the navigation task is done. <br/>

- ``BasicNavigator.getFeedback()``: <br/>
Return the navigation action feedback, to analyze the state of the task execution. <br/>

- ``BasicNavigator.getResult()``: <br/>
Return the final result of the navigation task, as SUCCEEDED or ABORTED <br/>

Look at [``nav2_test.py``](https://github.com/AlePuglisi/navigation-learning/blob/main/nav2-course/8-nav2-interaction/nav2_test.py) for a clearly commented example of these methods.<br/>
While [``nav2_test_template.py``](https://github.com/AlePuglisi/navigation-learning/blob/main/nav2-course/8-nav2-interaction/nav2_test_template.py) for a clean version of the script, without excessive comments. <br/>
You can easily use it as a template for your own project!

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

### 3. Run the Python Script
<image src=https://github.com/user-attachments/assets/fa6e8698-ae4f-4652-9136-8381cdb02011>

In a new Terminal, move to the directory where you create the Python script. <br/>

I manage the selection of the navigation task (initialization, single goal, waypoints) using command line arguments. <br/>
Every time you want to send a specific navigation task to Nav2, rerun the script with the appropriate argument: <br/>

  - #### Initialize the Pose:
  
    This will initialize **2D Pose Estimate**, as defined in nav2_test.py.<br/>
  
  > [!IMPORTANT]
  > Always start with this, to select the 2D Pose Estimate, fundamental for any upcoming navigation task. <br/>
  > Also, run it only once at the beginning, or the pose estimate will be messed up if the wrong location is sent. <br/>
    
  ```bash
  # Terminal 3
  python3 nav2_test.py initialize 
  ```

  - #### Send Single Nav2 Goal:
  
    This will start a **Nav2 Goal** navigation, towards the goal pose denied in nav2_test.py


   ```bash
     # Terminal 3
     python3 nav2_test.py goal 
   ```
  
  - #### Send Waypoints:
    
    This will start the **Waypoint Following** functionality, through the waypoints defined in nav2_test.py
    
   ```bash
    # Terminal 3
    python3 nav2_test.py waypoints 
   ```

## Conclusion 

Consider this lesson as a simple introduction to nav2_simple_commander Python API, explore the [documentation](https://docs.nav2.org/commander_api/index.html) to see all the available functionality! <br/>

If you want to make this ``nav2_test.py`` script even more general, use the arguments (accessed by ``sys.argv[]``) to define the Goal pose or the waypoints, instead of hardcoding it.<br/>
