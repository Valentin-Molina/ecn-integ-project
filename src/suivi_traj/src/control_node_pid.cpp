#include "control_node_pid.h"

using namespace std;

ControlNodePID::ControlNodePID(int controlled_limb): nh("~"), rate(10)
{

    joint_ok = false;

    if(controlled_limb == 0){
        torque_pub = nh.advertise<std_msgs::Float64>("/hand_effort_controller/command", 1);

    }
    else{
        torque_pub = nh.advertise<std_msgs::Float64>("/finger_effort_controller/command", 1);
    }


    joint_sub = nh.subscribe("/joint_states", 1, &ControlNodePID::readJointState, this);
    desired_sub = nh.subscribe("/Trajectoire", 1, &ControlNodePID::readTraj, this);

    last_state.position.push_back(0);
    last_state.position.push_back(0);
    last_state.velocity.push_back(0);
    last_state.velocity.push_back(0);
    desired_traj.position.push_back(0);
    desired_traj.position.push_back(0);
    desired_traj.velocity.push_back(0);
    desired_traj.velocity.push_back(0);

    torque.data = 0;
    controlledLimb = controlled_limb;
}

void ControlNodePID::readJointState(const sensor_msgs::JointStateConstPtr &_msg)
{
    joint_ok = true;
    last_state = *(_msg);

//    cout<<"CONTROL TALKING:"<<endl;
//    cout<<"state received: "<<endl;
//    cout<<"q1: "<<last_state.position[0]<<endl;
//    cout<<"q2: "<<last_state.position[1]<<endl;
//    cout<<"v1: " <<last_state.velocity[0]<<endl;
//    cout<<"v2: "<<last_state.velocity[1]<<endl;
//    cout<<"v2: "<<last_state.velocity[1]<<endl;

}

void ControlNodePID::setTorque()
{
    torque.data =Kp*(desired_traj.position[controlledLimb]-last_state.position[controlledLimb])+Kv*(desired_traj.velocity[controlledLimb]-last_state.velocity[controlledLimb])+Ki*0.1*(desired_traj.position[controlledLimb]-last_state.position[controlledLimb]);

    cout<<"Torque : "<<torque<<std::endl;

    torque_pub.publish(torque);
}

void ControlNodePID::readTraj(const sensor_msgs::JointStateConstPtr &_msg)
{
    desired_traj = *(_msg);
    cout<<"qDesired: "<<desired_traj.position[controlledLimb]<<endl;
}

bool ControlNodePID::ok()
{
    if( !nh.getParam("Kp",Kp) )
        {
            ROS_FATAL("Couldn't find parameter: Kp\n");
            return 0 ;
        }

    if( !nh.getParam("Kv",Kv) )
        {
            ROS_FATAL("Couldn't find parameter: Kv\n");
            return 0 ;
        }
    if( !nh.getParam("Ki",Ki) )
        {
            ROS_FATAL("Couldn't find parameter: Ki\n");
            return 0 ;
        }
    ros::spinOnce();
    rate.sleep();

    if(!(joint_ok))
        std::cout << "Waiting for incoming messages... did you start the simulation?\n";
    return joint_ok;
}
