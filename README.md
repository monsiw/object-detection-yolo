# Object Detection with Robotic Platform
An idea of this project is to implement the functionality object recognition on the Intel NUC10i5FNB computer, which is part of a car-like structure (image below). This robot moves using 4 wheels, where the two front wheels are controlled by a servo and the two rear wheels are powered by a motor with a nominal voltage of 12V each. The entire facility is powered by a 5000mAh Bashing lithium polymer battery by GensAce, which can be charged by connecting an external power supply to the dashboard. At the very top of the chassis, there is a HAMA camera for image recording. The board with the programmed controller accepts signals from the computer and distributes them to the wheels.<br/><br/>
<img src="https://github.com/monsiw/object_detection_yolov5/blob/main/images/3.PNG" width="400" height="200" />
## Start
In the workspace you need to build your packages using<br/>
*catkin_make*<br/>
*source devel/setup.bash*
## Inference with *darknet_ros* package
In order to start inference of chosen model write *roslaunch darknet_ros darknet_ros.launch* in the terminal<br/><br/>
<img src="https://github.com/monsiw/object_detection_yolov5/blob/main/images/1.png" width="400" height="200" /> <br/>
## Robot movement <br/>
*rosrun teleop_twist_keyboard teleop_twist_keyboard.py*<br/><br/>
The keys represent the following maneuvers: <br/>
• __*u*__  key is responsible for driving forward with the front wheels turned to the right, <br/>
• __*i*__  key is responsible for moving forward with the front wheels in the starting position, <br/>
• __*o*__  key is responsible for driving forward with the front wheels turned to the left, <br/>
• __*j*__  key is responsible for turning the front wheels to the right, <br/>
• __*k*__  key stops movement, <br/>
• __*l*__  key is responsible for turning the front wheels to the left, <br/>
• __*m*__  key is responsible for driving backwards with the front wheels turned to the right, <br/>
• __*,*__  key is responsible for driving backwards with the front wheels in the starting position, <br/>
• __*.*__  key is responsible for driving backwards with the front wheels turned to the left. <br/><br/>
<img src="https://user-images.githubusercontent.com/42692566/219169130-8e171972-6bff-45c8-b22f-fa87af437f7b.mp4" width="200" height="300"><br/>
<br/><br/>
With *rosrun rqt_graph rqt_graph* you should see the graph posted below <br/> <br/>
<img src="https://github.com/monsiw/object_detection_yolov5/blob/main/images/2.png" width="400" height="200" />
## Citing
[1] Arguedas M., et al.: ROS OpenCV camera driver – https://github.com/OTL/cv_camera. <br/>
[2] Baltovski T., et al.: teleop_twist_keyboard – https://github.com/ros-teleop/teleop_twist_keyboard. <br/>
[3] Bjelonic M.: YOLO ROS: Real-Time Object Detection for ROS – https://github.com/leggedrobotics/darknet_ros. <br/>
