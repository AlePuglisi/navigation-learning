#!/usr/bin/env python3 

# This is a clean version of nav2_test.py, without useless code and excessive comments. 
#!/usr/bin/env python3 

# This script can be easily integrated inside the node of a ros2 package. 

# The objective is to programmatically interact with Nav2 from external code, without using Rviz GUI: 
# This functionality can be used for more complex programs, where navigation goal decisions are chosen by high-level controllers, 
# like a vision-based system.

# import
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

import tf_transformations

import sys 

# This function returns a PoseStamped object, given the BasciNavigator object and the x, y, yaw pose (2D pose)
def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
        q_x, q_y, q_z, q_w =  tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z) # convert yaw orientation (euler angles) into quaternions
        pose = PoseStamped()                                      
        pose.header.frame_id = 'map'                             
        pose.header.stamp = navigator.get_clock().now().to_msg()  
        # Define position 
        pose.pose.position.x = position_x                   
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0         
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
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0) # we know that the robot spawn in the world origin, so at (0,0) with orientation 0 

    # sys.argv[1] is used to manage the desired task, run the code as:  python3 nav2_test.py <task>
    # 2. Use that pose to initialize the pose 
    # If you want to setInitialPose, launch the script with argument: initialize
    if sys.argv[1] == "initialize":
        print("Initialize Pose")         
        print(initial_pose)            
        nav.setInitialPose(initial_pose) 
    else: 
        print("NOT Initializing Pose")    
        
    # 3. Wait for Nav2
    nav.waitUntilNav2Active()        

    # --- Send Nav2 Goal ---
    
    # 1. Create and initialize a PoseStamped object (proceed as before)
  
    # One pose is enough if we go to one Goal
    goal_pose1 = create_pose_stamped(nav, 3.5, 1.0, 1.57)
    # Define multiple poses for Waypoint Follow
    goal_pose2 = create_pose_stamped(nav, 2.0, 2.5, 3.14)
    goal_pose3 = create_pose_stamped(nav, 0.5, 1.0, -1.57)

    # --- Go to One pose: ---
    # If you want to goToPose, launch the script with the argument: goal
    if sys.argv[1] == "goal":
        print("Single Goal Navigation")        
        # 2.a Use that pose to send the goal  
        nav.goToPose(goal_pose1)             

        # 3.a Wait for Nav2
        while not nav.isTaskComplete():     
            feedback = nav.getFeedback()   
            #print(feedback)
    
    # --- Follow Waypoints ---
    # If you want to followWaypoints, launch the script with the argument: waypoints
    if sys.argv[1] == "waypoints":
        print("Waypoint Follower Navigation")
        # 2.b Create an array of goal_pose, which is our waypoint goal
        waypoints = [goal_pose1, goal_pose2, goal_pose3]

        # 3.b Start waypoint follower
        nav.followWaypoints(waypoints)     
        # 3.b Wait for Nav2
        while not nav.isTaskComplete():     
            feedback = nav.getFeedback()    
            #print(feedback)

    # 4. Visualize Navigation final result
    print(nav.getResult())


    # --- Shutdown ros communication ---
    rclpy.shutdown() 


if __name__ == '__main__':
    main()
