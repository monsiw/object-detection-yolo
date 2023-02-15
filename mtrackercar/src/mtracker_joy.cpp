#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class MtJoy {
public:
  ros::Publisher  ctrl_pub;
  ros::Subscriber joy_sub;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    geometry_msgs::Twist controls;

    controls.linear.x  = 3 * joy_msg->axes[1];
    controls.angular.z = 5 * joy_msg->axes[0];

    ctrl_pub.publish(controls);
  }
};


int main(int argc, char **argv)
{
  MtJoy m;

  ros::init(argc, argv, "mtracker_joy");
  ros::NodeHandle n;

  ROS_INFO("MTracker joystick controller start");

  m.ctrl_pub = n.advertise<geometry_msgs::Twist>("/controls", 10);
  m.joy_sub = n.subscribe("/joy", 10, &MtJoy::joyCallback, &m);

  ros::spin();

  return 0;
}
