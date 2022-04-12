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

class PersonFollower(object):
    """ This node finds a person and follows them around """

    def __init__(self):
        # Initializing the ROS node
        rospy.init_node("person_follower")

        # Setting up the subscriber to the scan topic and
        # putting self.follow_person as the callback function.
        rospy.Subscriber("/scan", LaserScan, self.follow_person)

        # Setting up the publisher for the cmd_vel ROS topic
        self.movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # A default twist message for movement (all 0 values)
        self.movement = Twist(
            linear=Vector3(),
            angular=Vector3()
        )

        # A default distance that the robot should keep from the person
        self.goal_following_dist = 0.2

    def follow_person(self, data):
        '''
        Detect a person within the range of the robot using the scan data from
        everywhere around the robot (the person is assumed to be the closest detected obstacle), 
        turn towards that detected individual, then follow them around. 
        The robot should also stop at a distance away from the person so as not to get too close. 

        The ranges field in the scan data is a list of 360 degrees in the LiDAR scan;  each number
        in the list corresponds to the distance to the closest obstacle from the LiDAR.
        
        ranges[0] corresponds with what is directly in front of the robot, and as index to ranges
        increases by one it is a single degree in the counterclockwise direction.
        '''

        # FIRST, we want to identify at what angle relative to the robot the person is located.
        ranges = np.asarray(data.ranges)
        # Determine which angles in the ranges vector are detecting something at a nonzero distance
        nonzero_distances = np.logical_and(np.isfinite(ranges), ranges > 0)
        # If nothing in the LiDAR range is at a nonzero distance, then stop moving
        if sum(nonzero_distances) == 0: 
            # If so, then just stay still
            self.movement.linear.x = 0
            self.movement.angular.z = 0
        # Otherwise, determine where the person is located so that the robot can turn towards them
        else:
            # Let's tile up the 360 degree set of angles into slices of size 20 degrees, 
            # and see which 'slice' the person is closest to so that we are not basing ultimate 
            # angular direction commands on measurements from a noisy, single angle 
            slice_size = int(20) # in degrees

            # Initialize a vector that will hold the mean distance of environmental elements
            # from the range of angles in each slice
            slice_means = [] 

            # Set all the 0.0s (no detection) to NaN values so that we can take the 
            # mean distance of the slice without including 0.0s
            ranges[ranges == 0.0] = np.nan 

            # Iterate over all number of slices
            for s in range(int(len(ranges)/slice_size)):
                # Special case for the first slice, which needs to combine samples
                # from 0-10 degrees and 350-360 degrees (which are on opposite sides of the ranges vector)
                if s == 0:
                    first_half_angles = slice(0,int(slice_size/2))
                    second_half_angles = slice(int(-slice_size/2),0)
                    # Get the mean of the distances for all angles in the slice, disregarding NaNs (no detection)
                    slice_mean = np.nanmean(np.append(ranges[first_half_angles],ranges[second_half_angles]))
                else:
                    # Subsequent slices start from 10 degrees, so need to define slices offset by this amount
                    offset = slice_size/2
                    slice_angles = slice(int(offset + ((s-1)*slice_size)), int(offset + (s*slice_size)))
                    slice_mean = np.nanmean(ranges[slice_angles])

                slice_means.append(slice_mean)

            # If a person is closest to a given slice, we want the robot to move towards the 
            # angle at the *center* of that slice. Candidate goal angles are relative to the front of the robot (0 degrees). 
            # Positive angles are counterclockwise from the front, negative angles are clockwise from the front.
            candidate_goal_angles = [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, -160, -140, -120, -100, -80, -60, -40, -20]

            # We'll define the algorithm so that the robot follows the closest thing 
            # in its range (where the person is assumed to be in this situation)
            slice_facing_person = np.nanargmin(slice_means)
            
            distance_to_person = slice_means[slice_facing_person]
            angle_of_person = candidate_goal_angles[slice_facing_person]

            # Proportional control for angular motion to orient the robot so that
            # the front of the robot (0 degrees) reaches the goal of the angle the person is located.
            # If the person is on the right side of the robot, it rotates right.
            # If the person is on the left side of the robot, it rotates left.
            angle_error = angle_of_person
            angle_k_val = 0.01
            self.movement.angular.z = angle_k_val * angle_error

            # Proportional control for linear motion to make the robot
            # move to the person at a set following distance
            dist_error = distance_to_person - self.goal_following_dist
            dist_k_val = 0.2
            self.movement.linear.x = dist_k_val * dist_error 

        self.movement_pub.publish(self.movement)

    def run(self):
        # Keep the program running
        rospy.spin()

if __name__ == '__main__':
    # Declaring a node and then running it
    node = PersonFollower()
    node.run()
