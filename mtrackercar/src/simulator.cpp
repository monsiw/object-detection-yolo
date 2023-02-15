#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

class Simulator {
public:
  ros::Subscriber  ctrl_sub;
  ros::Publisher   pos_pub;

  geometry_msgs::Pose2D pos;

  void ctrlCallback(const geometry_msgs::Twist::ConstPtr& controls)
  {
    float v = controls->linear.x;
    float w = controls->angular.z;
  }

  void computePose()
  {
    // HERE PUT THE SIMULATOR CODE 

    pos.x = 0.0;
    pos.y = 0.0;
    pos.theta = 0.0;
  }
};


int main(int argc, char **argv)
{
  Simulator s;

  ros::init(argc, argv, "mtracker_simulator");
  ros::NodeHandle n;

  s.pos_pub = n.advertise<geometry_msgs::Pose2D>("/pos", 10);
  s.ctrl_sub = n.subscribe("/controls", 10, &Simulator::ctrlCallback, &s);

  ROS_INFO("MTracker Simulator");

  ros::Rate rate(100.0);

  while (ros::ok())
  {
    ros::spinOnce();

    s.computePose();
    s.pos_pub.publish(s.pos);

    rate.sleep();
  }

  return 0;
}
