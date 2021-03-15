#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "../include/trapezoidalNode.h"

void TrapezoidalNode::subscriberCallback(const geometry_msgs::Pose2D& msg)
{
    //ROS_INFO("I heard: ([%f],[%f])", msg.x, msg.y);
    buffer_.push_back(msg);

    Vector2f kv = {3.6,7};
    Vector2f ka = {0.1,8};

    for(auto pos : buffer_)
    {
        Vector2f qf;
        qf.x = pos.x;
        qf.y = pos.y;

        PlanTrajectory({0,0},qf,100,kv,ka);
    }
}

TrapezoidalNode::TrapezoidalNode()
{
    pubTimer_ = nh_.createTimer(ros::Duration(0.01), &TrapezoidalNode::timerCallback, this);
    sub_ = nh_.subscribe("Waypoints", 1000, &TrapezoidalNode::subscriberCallback, this); //To be replaced by the service bellow
    srv_ = nh_.advertiseService("Waypoint_serv", &TrapezoidalNode::serviceCallback, this);
    pub_ = nh_.advertise<sensor_msgs::JointState>("Trajectoire", 1000);
}

TrapezoidalNode::~TrapezoidalNode()
{
}

bool TrapezoidalNode::serviceCallback(trapezoidal_planning::WayPoint::Request& req, trapezoidal_planning::WayPoint::Response& res)
{
    // Example
    return true;
}

void TrapezoidalNode::timerCallback(const ros::TimerEvent& event)
{
    if(!currentTrajectory_.empty())
    {
        pub_.publish(currentTrajectory_[0]);
        currentTrajectory_.erase(currentTrajectory_.begin());
    }
}

void TrapezoidalNode::PlanTrajectory(const Vector2f& qi, const Vector2f& qf, float freq, Vector2f kv, Vector2f ka)
{
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

        currentTrajectory_.push_back(state);
        t+=dt;
    }

    denom = tf-tau;

    while(t < tf-tau)
    {
        sensor_msgs::JointState state;
        state.position = {qi.x + D.x*(2*t-tau)/(2*denom), qi.y + D.y*(2*t-tau)/(2*denom)};
        state.velocity = {D.x/denom, D.y/denom};
        state.effort = {0,0};

        currentTrajectory_.push_back(state);
        t+=dt;
    }

    denom = tau*(tf-tau);
    while (t < tf)
    {
        sensor_msgs::JointState state;
        state.position = {qi.x + D.x*(1-(tf-t)*(tf-t)/(2*denom)), qi.y + D.y*(1-(tf-t)*(tf-t)/(2*denom))};
        state.velocity = {D.x*(tf-t)/denom, D.x*(tf-t)/denom};
        state.effort = {-D.x/denom,-D.y/denom};

        currentTrajectory_.push_back(state);
        t+=dt;
    }
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "trapezoidal_planning");


  TrapezoidalNode TrapezoidalNode;
  
  ros::spin();

  return 0;
} // end main()
