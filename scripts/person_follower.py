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
        self.goal_following_dist = 0.5 # in meters

    def follow_person(self, data):
        # Detect a person within the range of the robot using the scan data from
        # everywhere around the robot, turn towards that detected individual, then 
        # follow them around. The robot should also stop at a distance away from the person
        # so as not to get too close. 

        # CHANGE SO THAT IT's MORE ME
        # The ranges field in the scan data is a list of 360 degrees in the LiDAR scan;  each number
        # corresponds to the distance to the closest obstacle from the LiDAR.
        
        # ranges[0] corresponds with what is directly in front of the robot, and as index to ranges
        # increases it is a 

        # IF PERSON IS DETECTED
            # go towards the angle with a range that is greater than self.distance but
            # has the minimum range ; in addition, it shouldn't be zero
        
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
            ranges[ranges == np.inf] = np.nan
            tile_up_by = int(20) # in degrees
            # goal angle is halfway in each pie slice
            possible_goals = [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, -160, -140, -120, -100, -80, -60, -40, -20]
            slice_means = []
            num_slice_elements = []

            print(len(ranges))
            for s in range(int(len(ranges)/tile_up_by)):
                if s == 0:
                    first_half = slice(0,int(tile_up_by/2))
                    second_half = slice(int(-tile_up_by/2),0)
                    elements_in_slice = sum(nonzero_distances[:int(tile_up_by/2)]) + sum(nonzero_distances[-1*int(tile_up_by/2):])
                    
                    slice_mean = np.nanmean(np.append(ranges[:int((tile_up_by/2))],ranges[:int((-tile_up_by/2))]))
                    
                else:
                    offset = tile_up_by/2
                    slice_idx = slice(int(offset + ((s-1)*tile_up_by)), int(offset + (s*tile_up_by)))
                    elements_in_slice = sum(nonzero_distances[slice_idx])
                    slice_mean = np.nanmean(ranges[slice_idx])
                    

                num_slice_elements.append(elements_in_slice)
                slice_means.append(slice_mean)
            print(num_slice_elements)
            print(slice_means)
            slice_with_most_detection = np.argmax(num_slice_elements)
            # this should be changed to one that has the most nonzero values
            smallest_distance = slice_means[slice_with_most_detection]
            # this should be the mean for that given slice, but increase the k-value for the proportional control
            smallest_angle = possible_goals[slice_with_most_detection]
            # 10 to -10
            # -10 to -30, +10 to +30
            # 30 to 50, 50 to 70, 70 to 90, 90 to 110, 110 to 130, 130 to 150, 150 to 170
            # 170 to -170
            # and so on until...
            # 170 to 180, -170 to -180
            # whichever has the greatest value, we set the goal angular velocity to the center

            # Angle the robot in the direction of the closest object, still nonzero
            #smallest_distance = np.min(ranges[nonzero_distances])
            #smallest_angle = np.where(ranges == smallest_distance)[0][0]
            print('smallest angle: ' + str(smallest_angle))

            # If the person is on the right side of the robot, we want to rotate right.
            # If the person is on the left side of the robot, we want it rotate left.
            # If it is within a range of values direction behind the robot (+/- 5 degrees from 180 degrees), 
            # we define that it will rotate left until it reaches that destination 
            # (to reduce the chance of noise impacting the angular movement decision)
            angle_error = smallest_angle

            angle_k = 0.005
                        
            self.movement.angular.z = angle_k * angle_error
            print("angular_accel: " + str(self.movement.angular.z)) 
            dist_error = smallest_distance - self.goal_following_dist
            print("smallest_dist: " + str(smallest_distance))
            print("error: " + str(dist_error))
            dist_k = 0.05
            self.movement.linear.x = dist_k * dist_error
            print("linear move: " + str(self.movement.linear.x)) 
        self.movement_pub.publish(self.movement)


    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = PersonFollower()
    node.run()
