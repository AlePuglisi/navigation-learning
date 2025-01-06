#!/usr/bin/env python3 

# This script can be easily integrated inside the node of a ros2 package. 
# This is just a test script, to learn how to use nav2_simple_commander API

# The objective is to programmatically interact with Nav2 from external code, without using Rviz GUI: 
# This functionality can be used for more complex programs, where navigation goal decisions are chosen by high-level controllers, 
# like a vision-based system.

# Here Initial pose (0,0,0) and goal_poses are defined in the code itself, you can easily initialize it when running the script.
# Consider using sys.argv[2],.. to define the initial pose, goal pose, or multiple waypoints...

# import
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

import tf_transformations

import sys 

# This function returns a PoseStamped object, given the BasciNavigator object and the x, y, yaw pose (2D pose)
# Because we are navigating in 2D worlds, 3 Degrees of Freedom are enough to define the pose
# Specifically, we identify robot pose from its position on the plane (x,y) and the orientation with respect to vertical z-axis (yaw angle)
def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
        q_x, q_y, q_z, q_w =  tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z) # convert yaw orientation (euler angles) into quaternions
        pose = PoseStamped()                                      # Initialize PoseStamped (instantiate) 
        pose.header.frame_id = 'map'                              # Relative frame of the pose 
        pose.header.stamp = navigator.get_clock().now().to_msg()  # Get time stamp from nav object
        # Fill in pose stamped pose attributes;
        # Define position 
        pose.pose.position.x = position_x                   
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0          # We are navigating in 2D (on XY plane), z = 0.0 always
        # Define orientation from quaternion
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w   

        return pose 
        
def main():
    # --- init --- 
    rclpy.init()                # Init ros communication
    nav = BasicNavigator()      # Init nav2 API 

    # --- Set initial Pose ---

    # # 1. Create and initialize a PoseStamped object
    # # Note: even if the InitialPose used by Nav2 is a PoseWithCovarianceStamped, 
    # # the API will make the conversion on its own before publishing it.  

    # # Convert orientation from euler to quaternion (adapt accordigly to orientation)
    # q_x, q_y, q_z, q_w =  tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)     
    # initial_pose = PoseStamped()                                
    # initial_pose.header.frame_id = 'map'                        # Relative frame of the pose 
    # initial_pose.header.stamp = nav.get_clock().now().to_msg()  # Get time stamp from nav
    # # Initialize position (adapt accordigly to position in map frame)
    # initial_pose.pose.position.x = 0.0                   
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.position.z = 0.0
    # # Initialize orientation 
    # initial_pose.pose.orientation.x = q_x
    # initial_pose.pose.orientation.y = q_y
    # initial_pose.pose.orientation.z = q_z
    # initial_pose.pose.orientation.w = q_w

    # We can use our create_pose_stamped function to do it in a single line: 
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0) # we know that the robot spawn in the world origin, so at (0,0) with orientation 0 

    # sys.argv[1] is used to manage the desired task, run the code as:  python3 nav2_test.py <task>
    # 2. Use that pose to initialize the pose 
    # If you want to setInitialPose, launch the script with argument: initialize
    if sys.argv[1] == "initialize":
        print("Initialize Pose")          # terminal information
        print(initial_pose)               # show pose defined 
        nav.setInitialPose(initial_pose)  # Interact with Nav2, initialize the pose 
    else: 
        print("NOT Initializing Pose")    
        
    # 3. Wait for Nav2
    # Before doing any other navigation task or shutting down the node, 
    # we have to wait for Nav2 to be ready, correctly initializing the pose
    nav.waitUntilNav2Active()        

    # --- Send Nav2 Goal ---
    
    # 1. Create and initialize a PoseStamped object (proceed as before)

    # q_x, q_y, q_z, q_w =  tf_transformations.quaternion_from_euler(0.0, 0.0, 1.57)    

    # goal_pose = PoseStamped()
    # goal_pose.header.frame_id = 'map'
    # goal_pose.header.stamp = nav.get_clock().now().to_msg()
    # # Define position 
    # goal_pose.pose.position.x = 3.5                   
    # goal_pose.pose.position.y = 1.0
    # goal_pose.pose.position.z = 0.0
    # # Define orientation 
    # goal_pose.pose.orientation.x = q_x
    # goal_pose.pose.orientation.y = q_y
    # goal_pose.pose.orientation.z = q_z
    # goal_pose.pose.orientation.w = q_w

    # We can use our create_pose_stamped function to do it in a single line: 
    # One pose is enough if we go to one Goal
    goal_pose1 = create_pose_stamped(nav, 3.5, 1.0, 1.57)
    # Define multiple poses for Waypoint Follow
    goal_pose2 = create_pose_stamped(nav, 2.0, 2.5, 3.14)
    goal_pose3 = create_pose_stamped(nav, 0.5, 1.0, -1.57)

    # --- Go to One pose: ---
    # If you want to goToPose, launch the script with the argument: goal
    if sys.argv[1] == "goal":
        print("Single Goal Navigation")        # Terminal info
        # 2.a Use that pose to send the goal  
        nav.goToPose(goal_pose1)              # Start navigation action, toward defined goal_pose 

        # 3.a Wait for Nav2
        while not nav.isTaskComplete():     # isTaskComplete return True when task sent is completed
            feedback = nav.getFeedback()    # getFeedback gives us the current robot position during the task 
            #print(feedback)
    
    # --- Follow Waypoints ---
    # If you want to followWaypoints, launch the script with the argument: waypoints
    if sys.argv[1] == "waypoints":
        print("Waypoint Follower Navigation") # Terminal info
        # 2.b Create an array of goal_pose, which is our waypoint goal
        waypoints = [goal_pose1, goal_pose2, goal_pose3]

        # 3.b Start waypoint follower
        nav.followWaypoints(waypoints)     # Start waypoint following action, across defined waypoints 
        # 3.b Wait for Nav2
        while not nav.isTaskComplete():     # isTaskComplete return True when task sent is completed
            feedback = nav.getFeedback()    # getFeedback gives us the current robot position during the task 
            #print(feedback)

    # 4. Visualize Navigation final result
    print(nav.getResult())


    # --- Shutdown ros communication ---
    rclpy.shutdown() 


if __name__ == '__main__':
    main()
