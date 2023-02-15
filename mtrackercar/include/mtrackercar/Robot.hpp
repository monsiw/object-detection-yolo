#pragma once

/* !ALL UNITS ARE IN SI! */

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "mtrackercar/Trigger.h"
#include "Serial.hpp"

#define ROBOT_LENGTH 0.200
#define ROBOT_MAX_STEERING M_PI/3.0
#define WHEEL_RADIUS 0.033

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


class Robot : public MtrackerSerial
{
  tf::TransformBroadcaster pos_broadcast;
  geometry_msgs::TransformStamped pos_tf;
   
public:
  geometry_msgs::Pose2D pos_odom;   // Position from odometry
  geometry_msgs::Twist  vel_odom;   // Velocity from odometry
  float rearWheelVel;            // rear wheel angular velocity
  float steeringAngle;
  bool motors_on;

  Robot() : rearWheelVel(0.0f), steeringAngle(0.0f), motors_on(true) {}
  ~Robot() {}

  void publishPose(ros::Publisher pub)
  {
    

    pos_tf.header.stamp = ros::Time::now();
    pos_tf.child_frame_id = "base";
    pos_tf.header.frame_id = "odom";
    pos_tf.transform.translation.x = pos_odom.x;
    pos_tf.transform.translation.y = pos_odom.y;
    pos_tf.transform.translation.z = 0.0;
    pos_tf.transform.rotation = tf::createQuaternionMsgFromYaw(pos_odom.theta);

    pos_broadcast.sendTransform(pos_tf);
    pub.publish(pos_odom);
  }

  void publishVelocity(ros::Publisher pub)
  {
    pub.publish(vel_odom);
  }

  void controlsCallback(const geometry_msgs::Twist msg)
  {
    auto v = rearWheelVel;
    auto w = msg.angular.z;
    
    rearWheelVel = msg.linear.x / WHEEL_RADIUS;
    
/*    if (v != 0)
    {
	steeringAngle = atan(ROBOT_LENGTH * w/v);
	
	if (abs(steeringAngle) > ROBOT_MAX_STEERING)
	   steeringAngle = ROBOT_MAX_STEERING * sgn(steeringAngle);
    }
    else
        steeringAngle = ROBOT_MAX_STEERING * sgn(w);    */
    steeringAngle = w;
  }

  bool triggerCallback(mtrackercar::Trigger::Request &req, mtrackercar::Trigger::Response &res)
  {
    motors_on = req.motors_on;
    return true;
  }
};

