#ifndef CONTROLNODEPID_HPP
#define CONTROLNODEPID_HPP

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <std_msgs/Float64.h>

class ControlNodePID
{
    public:
        ControlNodePID(int controlled_limb);

        // send a torque to the joints
        void setTorque();

        //checks if a message was received
        bool ok();

    protected:
            //to read q and qDot from the model
            ros::Subscriber joint_sub;
            
            //to read desired q and qDot from trajectory generator
            ros::Subscriber desired_sub;

            //to publish the torque
            ros::Publisher torque_pub;

            ros::NodeHandle nh;
            ros::Rate rate;

            //checks if a jointState msg has been recieved from simulation
            bool joint_ok;

    private:
        float Kp;
        float Kv;
        float Ki;

        int controlledLimb;

        //last q and qDot of the robot
        sensor_msgs::JointState last_state;
        //last q and qDot of the robot
        sensor_msgs::JointState desired_traj;

        //the Torque to be sent to the robot
        std_msgs::Float64 torque;

        void readJointState(const sensor_msgs::JointStateConstPtr &_msg);
        void readTraj(const sensor_msgs::JointStateConstPtr &_msg);
};


#endif // CONTROLNODEPID_HPP
