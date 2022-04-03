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
Some challenges for this first benchmark was just figuring out how to use rospy.spin() and also keep the periodic turning callback function to keep it driving in a square path. I think reading up on how ROS runs nodes and learning about potential issues like race conditions helped in conceptualizing how to solve the problem.

## Future Work
If I had more time I would like to set maybe different conditions (maybe based on user input when the program starts running) to specify in what direction we want the robot to drive in the square path. Would also like to experiment with the /odom topic as well.

## Takeaways
Overall this first part of the project helped me get more comfortable with how to setup/run a ROS node and how to navigate the ROS wiki for any resources. It also gave me practice with object oriented programming in Python, which will help in future assignments. 



