#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "../include/trapezoidalNode.h"


void TrapezoidalNode::subscriberCallback(const geometry_msgs::Pose2D& msg)
{
    ROS_INFO("I heard: ([%f],[%f])", msg.x, msg.y);
    buffer_.push_back(msg);
}

TrapezoidalNode::TrapezoidalNode()
{
    sub_ = nh_.subscribe("Waypoints", 1000, &TrapezoidalNode::subscriberCallback, this);
}

TrapezoidalNode::~TrapezoidalNode()
{
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "trapezoidal_planning");


  TrapezoidalNode TrapezoidalNode ;


  while(ros::ok())
  {}

  return 0;
} // end main()
