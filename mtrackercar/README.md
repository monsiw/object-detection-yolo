# MTrackerCar package for ROS #

This package contains several nodes for ROS system (Kinect) which serve as an interface for the user to use the robot. 

### The current nodes are: ###

* mtrackercar - main driver of the robot which connects to the low-level controller and sets the velocity and steering angle; publishes actual position and velocity of the robot
* mtracker_joy - translates the general joystick deflections to control signals
* controller - a closed loop controller for the robot
* simulator - simulates the physical robot and helps working offline, without the actual robot