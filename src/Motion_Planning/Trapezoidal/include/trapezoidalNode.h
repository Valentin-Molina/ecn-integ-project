#ifndef TRAPEZOIDAL_NODE_H
#define TRAPEZOIDAL_NODE_H

// ROS includes
#include<ros/ros.h>
#include<ros/time.h>
#include<vector>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

class TrapezoidalNode
{
public:
    TrapezoidalNode();
    ~TrapezoidalNode();
    void subscriberCallback(const geometry_msgs::Pose2D& msg);

private:
    std::vector<geometry_msgs::Pose2D> buffer_ ;

    ros::NodeHandle nh_ ;
    // Create a publisher and name the topic.
    ros::Subscriber sub_ ;

    //provisional : services will be used later
    float l1_, l2_ ; //arm lengths
    void mgd(const float &theta1, const float &theta2, float &x, float &y)
    {
        x = l1_*std::cos(theta1) + l2_*std::cos(theta1 + theta2);
        y = l1_*std::sin(theta1) + l2_*std::sin(theta1 + theta2);
    };
    void mgi(const float &x, const float &y, float &theta1, float &theta2, const bool &elbowLow = true)
    {
        theta2 = std::acos((l1_*l1_ + l2_*l2_)/(2*l1_*l2_));
        if(!elbowLow)
        {
            theta2 = - theta2 ;
        }
        theta1 = std::atan2(y, x) - theta2 ;
    };

};

#endif // TRAPEZOIDAL_NODE_H
