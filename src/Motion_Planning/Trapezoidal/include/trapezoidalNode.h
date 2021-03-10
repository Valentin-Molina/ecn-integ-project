#ifndef TRAPEZOIDAL_NODE_H
#define TRAPEZOIDAL_NODE_H

// ROS includes
#include<ros/ros.h>
#include<ros/time.h>
#include<vector>
#include <geometry_msgs/Pose2D.h>

class TrapezoidalNode
{
public:
    TrapezoidalNode();
    ~TrapezoidalNode();
    void subscriberCallback(const geometry_msgs::Pose2D& msg);
    std::vector<geometry_msgs::Pose2D> PlanTrajectory(const geometry_msgs::Pose2D& qi, const geometry_msgs::Pose2D& qf, float freq);

private:
    std::vector<geometry_msgs::Pose2D> buffer_ ;

    ros::NodeHandle nh_ ;
    // Create a publisher and name the topic.
    ros::Subscriber sub_ ;

};

#endif // TRAPEZOIDAL_NODE_H
