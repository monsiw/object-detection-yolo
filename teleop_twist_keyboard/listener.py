#!/usr/bin/env python
from __future__ import print_function
import threading
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import sys
from select import select
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty
TwistMsg = Twist
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""
def callback(msg):
      global x, theta
      x = msg.linear.x
      theta = msg.angular.z   
if __name__ == '__main__':
	rospy.init_node('listener')
	rospy.Subscriber("cmd_vel", TwistMsg, callback)
	rospy.spin()
