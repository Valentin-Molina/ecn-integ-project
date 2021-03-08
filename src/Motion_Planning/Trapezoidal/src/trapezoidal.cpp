#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "../include/trapezoidalNode.h"


void subscriberCallback(const geometry_msgs::Pose2D& msg)
{
	ROS_INFO("I heard: ([%f],[%f])", msg.x, msg.y);
}

TrapezoidalNode::TrapezoidalNode()
{
}

TrapezoidalNode::~TrapezoidalNode()
{
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "trapezoidal_planning");
  ros::NodeHandle nh;


  // Create a publisher and name the topic.
  ros::Subscriber sub = nh.subscribe("Waypoints", 1000, subscriberCallback);

  ros::spin();

  return 0;
} // end main()
