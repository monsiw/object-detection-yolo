#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include "math.h"
#include <sstream>
#include "vector"
#include <bits/stdc++.h>
#include <eigen3/Eigen/QR>   
#include <Eigen/Dense>

#ifndef NEWTON_HPP // include guard
#define NEWTON_HPP

class Newton
{
    public:
    float f_out[11], g_out[11], g_f_multiplication, cord[3];
    int max_iteration;
    std::vector < float > f_vector;
    std::vector < float > g_vector;

    void do_something();

    void f(float x_0[], float y_0[], int i);

    void g(float x_0[], float y_0[], int i);

    void newton_pinv(int i);
};


#endif /* MY_CLASS_H */