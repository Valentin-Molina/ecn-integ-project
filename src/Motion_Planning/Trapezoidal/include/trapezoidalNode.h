#ifndef TRAPEZOIDAL_NODE_H
#define TRAPEZOIDAL_NODE_H

// ROS includes
#include<ros/ros.h>
#include<ros/time.h>

class TrapezoidalNode
{
public:
    TrapezoidalNode();
    ~TrapezoidalNode();

    void publisherCallback(ros::Publisher, std::string);
};

#endif // TRAPEZOIDAL_NODE_H
