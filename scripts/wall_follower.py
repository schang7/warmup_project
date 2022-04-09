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
    """ This node finds a wall and follows them around """

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
        self.goal_following_dist = 0.25 # in meters

    def follow_wall(self, data):
        # Detect a wall within the range of the robot using the scan data from
        # everywhere around the robot, go towards that wall, then 
        # follow it. The robot should follow at a distance away from the wall
        # so as not to get too close. 

        # CHANGE SO THAT IT's MORE ME
        # The ranges field in the scan data is a list of 360 degrees in the LiDAR scan;  each number
        # corresponds to the distance to the closest obstacle from the LiDAR.
        
        # ranges[0] corresponds with what is directly in front of the robot, and as index to ranges
        # increases it is a 
        
        # Find all nonzero values
        ranges = np.asarray(data.ranges)
        nonzero_distances = np.logical_and(np.isfinite(ranges), ranges > 0)
        # Checking if everything in the LiDAR scan range is equal to inf or 0 
        # (in simulation)
        print(nonzero_distances)
        if sum(nonzero_distances) == 0:
            print('nothing is in range')
            # If so, then just stay still
            self.movement.linear.x = 0
            self.movement.angular.z = 0
        else:
            # Let's tile up the set of angles into sets of 20 degrees, so that we are not basing our angular direction command
            # on a single measurement
            ranges[ranges == 0.0] = np.nan
            tile_up_by = int(20) # in degrees
            # goal angle is halfway in each pie slice
            possible_goals = [10, 30, 50, 70, 90, 110, 130, 150, 170, -170, -150, -130, -110, -90, -70, -50, -30, -10]
            #possible_goals = [0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,-170,-160,-150,-140,-130,-120,-110,-100,-90,-80,-70,-60,-50,-40,-30,-20,-10]
            slice_means = []
            num_slice_elements = []

            print(len(ranges))
            for s in range(int(len(ranges)/tile_up_by)):
                offset = 0 
                slice_idx = slice(int(offset + (s*tile_up_by)), int(offset + ((s+1)*tile_up_by)))
                elements_in_slice = sum(nonzero_distances[slice_idx])
                slice_mean = np.nanmean(ranges[slice_idx])
                    
                num_slice_elements.append(elements_in_slice)
                slice_means.append(slice_mean)
            print(num_slice_elements)
            print(slice_means)
            #candidate_slices = np.flatnonzero(num_slice_elements == np.max(num_slice_elements))
            #candidate_slices = num_slice_elements
            #if len(candidate_slices) != 1:
            #    smallest_distance = 1000
            #    for i in candidate_slices:
            #        if slice_means[i] < smallest_distance:
            #            slice_with_most_detection = i
            #            smallest_distance = slice_means[i]
            #else:
            #    slice_with_most_detection = candidate_slices
            slice_with_most_detection = np.nanargmin(slice_means)
            smallest_distance = slice_means[slice_with_most_detection]
            
            # this should be the mean for that given slice, but increase the k-value for the proportional control
            smallest_angle = possible_goals[slice_with_most_detection]

            # Angle the robot such that the 
            #smallest_distance = np.min(ranges[nonzero_distances])
            #smallest_angle = np.where(ranges == smallest_distance)[0][0]
            print('smallest angle: ' + str(smallest_angle))

            # If the person is on the right side of the robot, we want to rotate right.
            # If the person is on the left side of the robot, we want it rotate left.
            # If it is within a range of values direction behind the robot (+/- 5 degrees from 180 degrees), 
            # we define that it will rotate left until it reaches that destination 
            # (to reduce the chance of noise impacting the angular movement decision)
            angle_error = smallest_angle - 90

            angle_k = 0.05

            # we've run into a wall and are following it now

            if smallest_distance < self.goal_following_dist: 
                print("went into smallest_dist")
                self.movement.angular.z = angle_k * angle_error
             
            # we're pulling away from a wall
            elif smallest_distance > self.goal_following_dist and smallest_angle > 90:
                self.movement.angular.z = angle_k * angle_error    
            else:
                self.movement.angular.z = 0 

            print("angular_accel: " + str(self.movement.angular.z)) 
            dist_error = smallest_distance - self.goal_following_dist
            #print("smallest_dist: " + str(smallest_distance))
            #print("dist_error: " + str(dist_error))
            dist_k = 0.5
            #self.movement.linear.x = 0.1
            self.movement.linear.x = 0.1
            print("linear move: " + str(self.movement.linear.x)) 
        self.movement_pub.publish(self.movement)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = WallFollower()
    node.run()





