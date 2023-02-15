#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

class Controller {
public:
  float x, y, theta;
  float t;
  geometry_msgs::Twist controls;

  ros::Publisher  ctrl_pub;
  ros::Subscriber pos_sub;

  void posCallback(const geometry_msgs::Pose2D::ConstPtr& pos_msg)
  {
    this->x = pos_msg->x;
    this->y = pos_msg->y;
    this->theta = pos_msg->theta;
  }

  void computeControls(float time)
  {

    // HERE PUT THE CODE

    auto vx = 0.1*sin(time);
    auto angle = 0.7*sin(2*time);

    controls.linear.x = vx;
    controls.angular.z = angle;
  }
};


int main(int argc, char **argv)
{
  Controller c;

  ros::init(argc, argv, "mtracker_controller");
  ros::NodeHandle n;

  c.ctrl_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  c.pos_sub = n.subscribe("/pos", 10, &Controller::posCallback, &c);

  ROS_INFO("MTracker Controller");

  ros::Rate rate(100.0);
  
  auto initTime = ros::Time::now().toSec();
  
  while (ros::ok())
  {
    ros::spinOnce();

    auto time = ros::Time::now().toSec() - initTime;

    c.computeControls(time);
    c.ctrl_pub.publish(c.controls);

    rate.sleep();
  }

  return 0;
}
