#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "../include/trapezoidalNode.h"
#include <stdlib.h>
#include <trapezoidal_planning/WayPoint.h>


TrapezoidalNode::TrapezoidalNode()
{
    freq_ = 1000.0;

    emittingTimer_ = nh_.createTimer(ros::Duration(1/freq_), &TrapezoidalNode::emittingCallback, this);
    computingTimer_ = nh_.createTimer(ros::Duration(1.0), &TrapezoidalNode::computingCallback, this);

    srv_ = nh_.advertiseService("Waypoint_serv", &TrapezoidalNode::serviceCallback, this);
    pub_ = nh_.advertise<sensor_msgs::JointState>("Trajectoire", 1000);

    kv = {6.0f,2.0f};
    ka = {2.0f,2.0f};



    float vMax1, vMax2, aMax1, aMax2;

    if(nh_.getParam("trapezoidal_planning/vMax1",vMax1))
        kv.x = vMax1;
    if(nh_.getParam("trapezoidal_planning/vMax2",vMax2))
        kv.y = vMax2;
    if(nh_.getParam("trapezoidal_planning/aMax1",aMax1))
        ka.x = aMax1;
    if(nh_.getParam("trapezoidal_planning/aMax2",aMax2))
        ka.y = aMax2;

    Vector2f initialPose = {0,0};
    buffer_.push_back(initialPose);
}


TrapezoidalNode::TrapezoidalNode(double _freq)
{
    freq_ = _freq;

    emittingTimer_ = nh_.createTimer(ros::Duration(1/freq_), &TrapezoidalNode::emittingCallback, this);
    computingTimer_ = nh_.createTimer(ros::Duration(1.0), &TrapezoidalNode::computingCallback, this);

    srv_ = nh_.advertiseService("Waypoint_serv", &TrapezoidalNode::serviceCallback, this);
    pub_ = nh_.advertise<sensor_msgs::JointState>("Trajectoire", 1000);

    kv = {6.0f,2.0f};
    ka = {2.0f,2.0f};



    float vMax1, vMax2, aMax1, aMax2;

    if(nh_.getParam("trapezoidal_planning/vMax1",vMax1))
        kv.x = vMax1;
    if(nh_.getParam("trapezoidal_planning/vMax2",vMax2))
        kv.y = vMax2;
    if(nh_.getParam("trapezoidal_planning/aMax1",aMax1))
        ka.x = aMax1;
    if(nh_.getParam("trapezoidal_planning/aMax2",aMax2))
        ka.y = aMax2;

    Vector2f initialPose = {0,0};
    buffer_.push_back(initialPose);
}


TrapezoidalNode::~TrapezoidalNode()
{

}

bool TrapezoidalNode::serviceCallback(trapezoidal_planning::WayPoint::Request& req, trapezoidal_planning::WayPoint::Response& res)
{
    if(isFree())
    {
        ROS_INFO("New trajectory added to the buffer. Computation will start...");
        res.ack = true ;
        int n = req.waypoints.size() ;
        Vector2f q;
        for(int i(0); i < n; i++)
        {
            q.x = req.waypoints[i].x ;
            q.y = req.waypoints[i].y ;
            buffer_.push_back(q) ;
        }
        return true ;
    }
    else
    {
        ROS_INFO("ERRROR : The current trajectory isn't finished.");
        res.ack = false ;
        return false ;
    }
}

bool TrapezoidalNode::isComputing()
{
    return !buffer_.empty();
}

bool TrapezoidalNode::isEmitting()
{
    return !currentTrajectory_.empty();
}

bool TrapezoidalNode::isFree()
{
    //return (!isEmitting() && !isComputing()); pb with the point (0, 0) at start
    return (!isEmitting());
}

void TrapezoidalNode::emittingCallback(const ros::TimerEvent& event)
{
    if(isEmitting())
    {
        //ROS_INFO("vMax = [%f],[%f] \naMax = [%f],[%f]",kv.x,kv.y,ka.x,ka.y);
        pub_.publish(currentTrajectory_[0]);
        currentTrajectory_.erase(currentTrajectory_.begin());
    }
}

void TrapezoidalNode::computingCallback(const ros::TimerEvent& event)
{
    if(isComputing())
    {
        PlanTrajectoryFromWaypointsBuffer(freq_);
    }
}

void TrapezoidalNode::PlanTrajectory(const Vector2f& qi, const Vector2f& qf, float freq)
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

void TrapezoidalNode::PlanTrajectoryFromWaypointsBuffer(float freq)
{
	//This method computes the all trajectory given by all the Waypoints in the buffer_, then empty the buffer_ except one position.
	//This position is then used as a starting point the next time we call this method.
	
    if(buffer_.size() > 1)
    {
        for(unsigned int i = 0; i < buffer_.size()-1; i++)
        {
            PlanTrajectory(buffer_[i],buffer_[i+1],freq);
        }

        buffer_.erase(buffer_.begin(),buffer_.end()-1);
    }
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "trapezoidal_planning");


  TrapezoidalNode TrapezoidalNode(1000.0);
  
  ros::spin();

  return 0;
} // end main()
