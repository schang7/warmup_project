#!/usr/bin/env python3

# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the wall is

import rospy
import numpy as np

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class WallFollower(object):
    """ This node finds a wall and follows along it """

    def __init__(self):
        # Initializing the ROS node
        rospy.init_node("wall_follower")

        # Setting up the subscriber to the scan topic and
        # putting self.follow_wall as the callback function.
        rospy.Subscriber("/scan", LaserScan, self.follow_wall)

        # Setting up the publisher for the cmd_vel ROS topic
        self.movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # A default twist message for movement (all 0 values)
        self.movement = Twist(
            linear=Vector3(),
            angular=Vector3()
        )

        # A default distance that the robot should keep from the wall
        self.goal_following_dist = 0.38

    def follow_wall(self, data):
        '''
        Moves the robot, and detects when it comes near contact with a wall robot using the scan data from
        everywhere around the robot. Once it reaches the wall it follows alongside the wall at an approximate
        set distance by changing the angular acceleration to minimize the distance of the left side of the robot
        from the wall. 

        The ranges field in the scan data is a list of 360 degrees in the LiDAR scan;  each number
        in the list corresponds to the distance to the closest obstacle from the LiDAR.
        
        ranges[0] corresponds with what is directly in front of the robot, and as index to ranges
        increases by one it is a single degree in the counterclockwise direction.
        '''
        # FIRST, make the robot move forward until it finds a wall
        self.movement.linear.x = 0.1

        # NEXT, we want to identify where any obstacles (e.g. a wall) is located relative to the robot.
        ranges = np.asarray(data.ranges)

        # Let's tile up the 360 degree set of angles into slices of size 20 degrees, 
        # and see which 'slice' has obstacles closest to the robot (so that we are not basing ultimate 
        # angular direction commands on measurements from a noisy, single angle)
        slice_size = int(20) # in degrees

        # Initialize a vector that will hold the mean distance of environmental elements
        # from the range of angles in each slice
        slice_means = [] 

        # Set all the 0.0s (no detection) to NaN values so that we can take the 
        # mean distance of the slice without including 0.0s
        ranges[ranges == 0.0] = np.nan 

        # Iterate over all number of slices
        for s in range(int(len(ranges)/slice_size)):
            # Each slice is from 0-20 degrees, 20-40 degrees, etc.
            slice_angles = slice(int(s*slice_size), int((s+1)*slice_size))
            slice_mean = np.nanmean(ranges[slice_angles])
                
            slice_means.append(slice_mean)

        # If obstacles are closest within a given slice, we want the robot to receive movement commands based on the 
        # angle at the *center* of that slice. Candidate goal angles are relative to the front of the robot (0 degrees). 
        # Positive angles are counterclockwise from the front, negative angles are clockwise from the front.
        candidate_goal_angles = [10, 30, 50, 70, 90, 110, 130, 150, 170, -170, -150, -130, -110, -90, -70, -50, -30, -10]

        closest_slice = np.nanargmin(slice_means)
        
        closest_distance = slice_means[closest_slice]
        closest_angle = candidate_goal_angles[closest_slice]
        

        # Proportional control for angular motion to orient the robot so that
        # the left side of the robot (90 degrees) should become the closest to the obstacle (the wall)
        angle_error = closest_angle - 90
        angle_k_val = 0.02

        # Handling different wall-contact cases:
        # When approaching the wall, if the closest distance to the wall
        # is less than the goal_following_dist then you need to turn to avoid a collision; 
        # this condition works when approaching a wall for the first time or turning inside
        # corners (e.g. in a square room)
        if closest_distance < self.goal_following_dist: 
            self.movement.angular.z = angle_k_val * angle_error
        # Don't move forward while this turn is happening; this helps a bit with preventing collisions    
        if closest_distance < self.goal_following_dist and closest_angle != 90:
            self.movement.linear.x = 0
        # If you have been following a wall but are starting to move away from it (e.g. at an
        # outside corner) so that the left side is no longer closest to the wall, you need to turn 
        elif closest_distance > self.goal_following_dist and closest_angle > 90:
            self.movement.angular.z = angle_k_val * angle_error  
        # Do not turn if you are still finding a wall or in the process of following a wall  
        else:
            self.movement.angular.z = 0 

        self.movement_pub.publish(self.movement)

    def run(self):
        # Keep the program running
        rospy.spin()

if __name__ == '__main__':
    # Declaring a node and then running it
    node = WallFollower()
    node.run()
