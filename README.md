# Object Detection with Robotic Platform
*rosrun rqt_graph rqt_graph* <br/> <br/>
<img src="https://github.com/monsiw/object_detection_yolov5/blob/main/images/2.png" width="400" height="200" />
## Inference with *darknet_ros* package
*roslaunch darknet_ros darknet_ros.launch*<br/><br/>
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
<img src="https://user-images.githubusercontent.com/42692566/219169130-8e171972-6bff-45c8-b22f-fa87af437f7b.mp4" width="200" height="300">


