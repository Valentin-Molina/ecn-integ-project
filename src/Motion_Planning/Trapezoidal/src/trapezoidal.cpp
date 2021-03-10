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

std::vector<sensor_msgs::JointState> TrapezoidalNode::PlanTrajectory(const Vector2f& qi, const Vector2f& qf, float freq, Vector2f kv, Vector2f ka)
{
    std::vector<sensor_msgs::JointState> ans;

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

    float tf = lambda1*kv.x/(mu1*ka.x) + std::abs(D.x)/(lambda1*kv.x);
    float tau = lambda1*kv.x/(mu1*ka.x);

    float t = 0;
    float dt = 1/freq;

    float denom = tau*(tf-tau);

    while(t <= tau)
    {
        sensor_msgs::JointState state;
        state.position = {qi.x + D.x*t*t/(2*denom), qi.y + D.y*t*t/(2*denom)};
        state.velocity = {D.x*t/denom, D.y*t/denom};
        state.effort = {D.x/denom,D.y/denom};

        ans.push_back(state);
        t+=dt;
    }

    denom = tf-tau;

    while(t < tf-tau)
    {
        sensor_msgs::JointState state;
        state.position = {qi.x + D.x*(2*t-tau)/(2*denom), qi.y + D.y*(2*t-tau)/(2*denom)};
        state.velocity = {D.x/denom, D.y/denom};
        state.effort = {0,0};

        ans.push_back(state);
        t+=dt;
    }

    denom = tau*(tf-tau);
    while (t < tf)
    {
        sensor_msgs::JointState state;
        state.position = {qi.x + D.x*(1-(tf-t)*(tf-t)/(2*denom)), qi.y + D.y*(1-(tf-t)*(tf-t)/(2*denom))};
        state.velocity = {D.x*(tf-t)/denom, D.x*(tf-t)/denom};
        state.effort = {-D.x/denom,-D.y/denom};

        ans.push_back(state);
        t+=dt;
    }

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
