#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "gkd_models/Dynamic.h"
#include <vector>
#include <std_msgs/Float64.h>

class ControlNode
{
    public:
        ControlNode();

        // send a torque to the joints
        void setTorque();

        //checks if a message was received
        bool ok();

        //checks if all params are assigned to values
        bool checkParams();

    protected:
            //to read q and qDot from the model
            ros::Subscriber joint_sub;
            
            //to read desired q and qDot from trajectory generator
            ros::Subscriber desired_sub;

            //to publish the torque
           ros::Publisher torque_pub[2];

            ros::NodeHandle nh;
            ros::Rate rate;

            //checks if a jointState msg has been recieved from simulation
            bool joint_ok;

    private:
        float Kp1;
        float Kv1;

        float Kp2;
        float Kv2;
        
        //represents qDD+Kp*e+Kv*eD
        std::vector<float> term;

        //last q and qDot of the robot
        sensor_msgs::JointState last_state;
        //last q and qDot of the robot
        sensor_msgs::JointState desired_traj;

        //the Torque to be sent to the robot
        //first one is the hand's the other is the finger's
        std_msgs::Float64 torque[2];

        gkd_models::Dynamic srv;

        void readJointState(const sensor_msgs::JointStateConstPtr &_msg);
        void readTraj(const sensor_msgs::JointStateConstPtr &_msg);
        void calculateTorqueTerm(const sensor_msgs::JointStateConstPtr &_msg);
		ros::ServiceClient client;

};


#endif // CONTROL_NODE_HPP
