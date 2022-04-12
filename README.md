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

## Person follower

### Description:
This ROS node commands the Turtlebot3 robot to follow a person. In my approach I first had the robot detect the location of the closest thing in its vicinity (assuming that is a person) using LiDAR scan data and then turn itself angularly towards that obstacle (using proportional control) so that the front of the robot faces the person. While angling towards the person, it also moves linearly towards them at a set following distance (also using proportional control) so that it does not collide with them.

### Code Explanation:
The node is defined in the class `PersonFollower`, which contains the following functions:

`__init__(self)`: the ROS node, publisher to the /cmd_vel topic, and subscriber to the /scan topic is initialized with the name of the `follow_person` callback function. In addition, an instance attribute containing a default Twist message specifying the linear and angular acceleration of the robot (set to 0 in the x, y, z directions is initialized, along with a goal following distance for the robot when it is approaching the person.

`follow_person(self, data)`: This is a callback function that makes the robot move towards a person in response to LiDAR scan data. It first processes the ranges field in the scan data, which is the list of 360 numbers corresponding to degrees in the LiDAR scan; each value in the list corresponds to the distance to the closest obstacle from the LiDAR at that angle (ranges[0] corresponds with what is directly in front of the robot, and the angle values increase counterclockwise from the front of the robot). If nothing is detected in the range of the robot, it does not move. 

However, if there are elements, then the LiDAR range is divided into slices that are 20 degrees in size (starting from -10 to 10 degrees, 10 to 30 degrees, etc.). The mean distance among the angles in a given zone are calculated for all the zones, and the one that has the smallest distance value is considered to be the slice facing the person. Using proportional control, the goal angle that the robot should turn itself towards to face the person is the angle at the center of this closest slice (e.g. for 10 to 30 degrees, turn towards 20 degrees). The error for proportional control in this case is this center angle value since we want the front of the robot to be facing the center angle. This center angle value is multiplied by a k-value and then set as the angular z-acceleration. Note that if this center angle is within a zone that contains angles within 180 degrees clockwise from the front of the robot, it should be negative (i.e. angles are within 0 to +180 or 0 to -180) so that the robot knows to turn left or right with a positive or negative angular z-acceleration respectively. 

Finally, this code also uses proportional control to control the linear acceleration of the robot. It takes the mean distance calculated from the zone closest to the person and sets the linear x acceleration to a k-value times the error of this “closest distance - the goal following distance”. 

`run(self)`: In this function, the program is kept running with the rospy.spin() command. 

### Gif of movement:

![](https://github.com/schang7/warmup_project/blob/7c73a98a3588d76bc39d0e3232fc01c1817fcf7a/gifs/person_follower.GIF)

## Wall follower

### Description:
This ROS node commands the Turtlebot3 robot to follow a wall. In my approach I first had the robot move around at a set, constant linear acceleration and detect a wall using LiDAR scan data. If it is within a set following distance from the wall, it turns itself angularly using proportional control so that the left side of the robot (90 degrees counterclockwise from the front of the robot)  is closest to the wall and it avoids collision. It then moves forward and follows that wall until it encounters an inside corner (in which it turns again to avoid collision with the wall) or starts to pull away from a wall (in which it turns towards the wall to continue following it). 

### Code Explanation:
The node is defined in the class `WallFollower`, which contains the following functions:

`__init__(self)`: the ROS node, publisher to the /cmd_vel topic, and subscriber to the /scan topic is initialized with the name of the `follow_wall` callback function. In addition, an instance attribute containing a default Twist message specifying the linear and angular acceleration of the robot (set to 0 in the x, y, z directions is initialized, along with a goal following distance for the robot approaching a wall.

`follow_wall(self, data)`: This is a callback function that makes the robot follow a wall in response to LiDAR scan data. The linear x acceleration of the robot is first set to a constant value so that the robot will move around until it encounters a wall. Then, the code processes the ranges field in the scan data, which is a list of 360 numbers corresponding to degrees in the LiDAR scan; each value in the list corresponds to the distance to the closest obstacle from the LiDAR at that angle (ranges[0] corresponds with what is directly in front of the robot, and the angle values increase counterclockwise from the front of the robot). 

The LiDAR distances in the range of 360 degrees are divided into slices that are 20 degrees in size (starting from 0 to 20 degrees, 20 to 40 degrees, etc.). The mean distance among the angles in a given zone are calculated for all the zones, and the one that has the smallest distance value is considered to be the one closest to an obstacle (a wall). Using proportional control, the code specifies that the robot turn itself such that its left side (at 90 degrees) faces the angle at the center of this closest slice (e.g. for 0 to 20 degrees, the center angle is 10 degrees). The error for proportional control in this case is the center angle - 90 since we want the left side of the robot to be closest to the wall. 

However, there are a few conditions that specify when the robot should be making these turns when it is first encountering the wall or while it is continuously following the wall. 
- When approaching a wall for the first time or approaching an inside corner of a wall, it should turn according to the proportional control command if the mean distance of the closest slice is less than the set goal following distance (so that it does not collide with the wall). In addition, the linear x velocity should be set to zero during this correcting turn. 
- Once it is following a wall with its left side closest to the wall, the angular z velocity would be set to 0 because the error for proportional control would be zero (90 - 90 = 0). Thus no turning movements are made.
- However, if the robot starts to pull away from the wall such that it is farther away from the wall than the set following distance and the angle closest to the wall is no longer 90 degrees, then it should also start turning again to continue following the wall.

`run(self)`: In this function, the program is kept running with the rospy.spin() command. 

### Gif of movement:

PART 1:

![](https://github.com/schang7/warmup_project/blob/7c73a98a3588d76bc39d0e3232fc01c1817fcf7a/gifs/wall_follower_part_1.GIF)

PART 2:

![](https://github.com/schang7/warmup_project/blob/7c73a98a3588d76bc39d0e3232fc01c1817fcf7a/gifs/wall_follower_part_2.GIF)


## Challenges
Some challenges for the first `drive_square` benchmark was just figuring out how to use rospy.spin() and also keep the periodic turning callback function running to have the robot continuously drive in a square path. I think reading up on how ROS runs nodes and learning about potential issues like race conditions helped me in conceptualizing how to solve these challenges.

A challenge that I faced coding `person_follower` and `wall_follower` was handling noisy measurements from the scan data, since the robot's movements based on noisy measurements would be very jittery or incorrect. I overcame this challenge by averaging out distance measurements across slices of angles in 360 degrees space and basing my angular movement commands on these averaged measurements. I also ran into problems setting up the right k-values for proportional control, and overcame this with a lot of trial and error and learning about the physical restrictions of the robot. For instance, I was confused about why the robot wasn't moving the way I expected even though it had a nonzero angular/linear acceleration, but it turned out that the specified acceleration value was too large for the robot to physically turn/move forward. 

Another challenge that I faced coding `wall_follower` was just trying to reason through all the different edge cases for encountering and following a wall. I overcame this challenge by drawing out a lot of the potential walls/corners that a robot could encounter and thinking about how the robot should move in each of those situations. 

## Future Work
If I had more time with the `drive_square` behavior I would like to set maybe different conditions (maybe based on user input when the program starts running) to specify in what direction we want the robot to drive in the square path. I would also like to experiment with the /odom topic as well.

If I had more time with the `person_follower` behavior I'd like to figure out how it's possible to make the robot specifically follow a person-like object instead of using the closest obstacle as a proxy for the person. I'd also like to make the following movements more smooth while also being faster and further fine tune the reduction of noise (through my slicing) in the ranges measurements. 

If I had more time with the `wall_follower` it would be to make it a more smooth turning manuever (doing more experimentation with k-values or adding more conditions) because the way that I have set up my conditions makes the robot a little jerky sometimes with the turning. Also, while it also does really well with square walls with very nice boundaries, sometimes with more crampled (acute) inside angle corners it can get close to the wall (graze it) or sometimes get too far away from the wall after obtuse outside angle corners. While some of this may be due to noise/friction in the physical movement of the robot, I think I would improve the way that the goal following distance from the wall is regulated (maybe with some non-constant linear acceleration modulations).

## Takeaways
 
- Overall, this project helped me get more comfortable with how to setup and run a ROS node and how to navigate the ROS wiki for any resources. It also gave me practice with object oriented programming in Python, which will help in future assignments. 
- This project also gave me a lot of practice with uses of proportional control. I became increasingly proficient at figuring out what are the goal or current states for different forms of angular or linear movement. I also learned a lot about choosing the right k-values to make the robot receive commands that are within its physical hardware constraints and also demonstrate the desired behavior. Using this skill will be key to helping me program future robot behaviors with sensorimotor control.
- I also got a lot of experience thinking about handling noisy measurements in real life situations. After writing up stuff in simulation, I realized how different it is when trying it on the physical turtlebots. Therefore, I practice in thinking about how to implement more robust code to handle these situations, which will be crucial in programming more robot behavior in real life scenarios.

