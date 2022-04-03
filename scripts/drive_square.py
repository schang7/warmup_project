#!/usr/bin/env python3

import rospy
import math

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):
    """ This node makes the robot drive in a square """
    def __init__(self):
        # Initializing the ROS node
        rospy.init_node('drive_in_square')
        # Setting up the publisher for the cmd_vel ROS topic
        self.movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # A default twist message for movement (all 0 values)
        self.movement = Twist(
            linear=Vector3(),
            angular=Vector3()
        )

    def turn_in_square(self, event):
        # A callback function which makes the robot turn in the square, 
        # is called every time the robot traverses the length of one "side" 
        # of the square in its driving path

        # First stop the robot's forward motion if it is moving,
        # then initiate angular movement for turning to the right 
        self.movement.linear.x = 0
        self.movement.angular.z = 1.0

        # Give it some time to register
        rospy.sleep(1)
        self.movement_pub.publish(self.movement)

        # Determine duration needed to complete 90 degree turn
        radians = math.pi/2 # to make a 90 degrees turn (in radians)
        duration_to_turn = radians / self.movement.angular.z

        # Turn for that amount of time
        rospy.sleep(duration_to_turn)

        # Stop moving in the angular direction, start moving forward again
        self.movement.angular.z = 0
        self.movement.linear.x = 0.5
        self.movement_pub.publish(self.movement)

    def run(self):
        # Set a timer to execute the turn_in_square callback function every time
        # a certain amount of seconds have elapsed
        time_before_turn = 5 # in seconds
        rospy.Timer(rospy.Duration(time_before_turn), self.turn_in_square)
        rospy.spin()

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()
