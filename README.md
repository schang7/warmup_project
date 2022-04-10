# Warmup Project Writeup

## Driving in a square

### Description:
This ROS node commands the Turtlebot3 robot to drive in a square path. In my approach
I took advantage of timing to make the robot move forward and periodically turn 90 degrees up to four times in order to complete a square path. 

### Code Explanation:
The node is defined in the class `DriveSquare`, which contains the following functions:

`__init__(self)`: the ROS node and the publisher to the /cmd_vel topic is initialized, along with an instance attribute containing a default Twist message specifying the linear and angular acceleration of the robot (set to 0 in the x, y, z directions).

`turn_in_square(self, event)`: This is a callback function that makes the robot turn 90 degrees and then move forwards again. It first sets the robot's linear x velocity to zero (to stop forward movement) and then the angular z velocity to a nonzero value. After the turn is completed the angular z velocity is set to zero and the linear x velocity is set to a non-zero value to initiate forwards movement again in a new "side" of the square path.

`run(self)`: In this function, the `turn_in_square` callback function is run in a periodic way using the rospy.Timer() function, which specifies to the robot that it should run the callback after at a specified time interval. This cycle repeats until the user quits the program. 

### Gif of movement:

![](https://github.com/schang7/warmup_project/blob/851ee79972db51cb5c71b61d9367554a4b457123/drive_square.GIF)

## Challenges
Some challenges for the first `drive_square` benchmark was just figuring out how to use rospy.spin() and also keep the periodic turning callback function running to have the robot continuously drive in a square path. I think reading up on how ROS runs nodes and learning about potential issues like race conditions helped me in conceptualizing how to solve these challenges.

A challenge that I faced coding `person_follower` and `wall_follower` was handling noisy measurements from the scan data, since robots movements based on noisy measurements would be very jittery or incorrect. I overcame this challenge by averaging out distance measurements across slices of angles in 360 degrees space and basing my angular movement commands on averaged distance measurements.

Another challenge that I faced coding `wall_follower` was just trying to reason through all the different edge cases for encountering and following a wall. I overcame this challenge by drawing out a lot of the potential walls/corners that a robot could encounter and thinking about how the robot should move in each of those scenarios. 

## Future Work
If I had more time with the `drive_square` behavior I would like to set maybe different conditions (maybe based on user input when the program starts running) to specify in what direction we want the robot to drive in the square path. I would also like to experiment with the /odom topic as well.

If I had more time with the `person_follower` behavior I'd like to figure out how it's possible to make the robot specifically follow a person-like object instead of using the closest obstacle as a proxy for the person. I'd also like to make the movements more smooth and further fine tune the reduction of noise (through my slicing) in the ranges measurements. 

If I had more time with the `wall_follower` it would be to ______.

## Takeaways
 
- Overall, this project helped me get more comfortable with how to setup and run a ROS node and how to navigate the ROS wiki for any resources. It also gave me practice with object oriented programming in Python, which will help in future assignments. 
- This project also gave me a lot of practice with uses of proportional control. I became increasingly proficient at figuring out what are the goal or current states for different forms of angular or linear movement. Using this skill will be key to helping me program future robot behaviors with sensorimotor control.
- I also got a lot of experience thinking about handling noisy measurements in real life situations. After writing up stuff in simulation, I realized how different it is when trying it on the physical turtlebots. Therefore, I practice in thinking about how to implement more robust code to handle these situations, which will be crucial in programming more robot behavior in real life scenarios.



