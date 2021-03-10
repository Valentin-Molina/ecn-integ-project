#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "../include/trapezoidalNode.h"

struct Vector2f
{
    float x;
    float y;
};

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

std::vector<Vector2f> TrapezoidalNode::PlanTrajectory(const Vector2f& qi, const Vector2f& qf, float freq, Vector2f kv, Vector2f ka)
{
    std::vector<Vector2f> ans;

    Vector2f D;
    D.x = qf.x-qi.x;
    D.y = qf.y-qi.y;

    if(std::abs(D.x)<kv.x*kv.x/ka.x)
    {
        kv.x=std::sqrt(std::abs(D.x)*ka.x);
    }

    if(std::abs(D.y)<kv.y*kv.y/ka.y)
    {
        kv.y=std::sqrt(std::abs(D.y)*ka.y);
    }

    float lambda1 = std::min(1.0f,kv.y*std::abs(D.x)/(kv.x*std::abs(D.y)));
    float mu1 = std::min(1.0f,ka.y*std::abs(D.x)/(ka.x*std::abs(D.y)));

//    float lambda2 = lambda1*kv.x*std::abs(D.y)/(kv.y*std::abs(D.x));
//    float mu2 = mu1 * ka.x*std::abs(D.y)/(ka.y*std::abs(D.x));

    float tf = lambda1*kv.x/(mu1*ka.x) + std::abs(D.x)/(lambda1*kv.x);

    return ans;
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "trapezoidal_planning");


  TrapezoidalNode TrapezoidalNode;
  
  ros::spin();

  return 0;
} // end main()
