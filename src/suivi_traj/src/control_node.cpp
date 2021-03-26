#include "control_node.h"

using namespace std;

ControlNode::ControlNode(): nh("~"), rate(10)
{
    Kp1 = 0;
    Kv1 =0;

    joint_ok = false;
    torque_pub[0] = nh.advertise<std_msgs::Float64>("/hand_effort_controller/command", 1);
    torque_pub[1] = nh.advertise<std_msgs::Float64>("/finger_effort_controller/command", 1);
    joint_sub = nh.subscribe("/joint_states", 1, &ControlNode::readJointState, this);
    desired_sub = nh.subscribe("/Trajectoire", 1, &ControlNode::calculateTorqueTerm, this);
    client     = nh.serviceClient<gkd_models::Dynamic>("Dynamic");

    last_state.position.push_back(0);
    last_state.position.push_back(0);
    last_state.velocity.push_back(0);
    last_state.velocity.push_back(0);
    desired_traj.position.push_back(0);
    desired_traj.position.push_back(0);
    desired_traj.velocity.push_back(0);
    desired_traj.velocity.push_back(0);

    torque[0].data = 0;
    torque[1].data = 0;

    term.push_back(0);
    term.push_back(0);
}

void ControlNode::calculateTorqueTerm(const sensor_msgs::JointStateConstPtr &_msg)
{
    cout<<"In term calculation, Torque:"<<endl;
    desired_traj = (*_msg);
    term[0] = desired_traj.effort[0]+Kp1*(desired_traj.position[0]-last_state.position[0])+Kv1*(desired_traj.velocity[0]-last_state.velocity[0]);
    term[1] = desired_traj.effort[1]+Kp2*(desired_traj.position[1]-last_state.position[1])+Kv2*(desired_traj.velocity[1]-last_state.velocity[1]);
}

void ControlNode::readJointState(const sensor_msgs::JointStateConstPtr &_msg)
{
    cout<<"joint read"<<endl;
    joint_ok = true;
    last_state = *(_msg);
}

void ControlNode::setTorque()
{ 
    srv.request.input = desired_traj;
    client.call(srv);
    torque[0].data = srv.response.output.effort[0];
    torque[1].data = srv.response.output.effort[1];

//    cout<<"RECEIVED FROM SERVICE:"<<endl;
//    cout<<srv.response.output.effort[0]<<endl;
//    cout<<srv.response.output.effort[1]<<endl;

    torque[0].data += term[0];
    torque[1].data += term[1];

    int nb =0;

    for (auto &i: torque_pub)
    {
        i.publish(torque[nb]);
        nb ++;
    }
}

bool ControlNode::checkParams()
{
    if( !nh.getParam("Kp1",Kp1) )
        {
            ROS_FATAL("Couldn't find parameter: Kp1\n");
            return 0;
        }

    if( !nh.getParam("Kv1",Kv1) )
        {
            ROS_FATAL("Couldn't find parameter: Kv1\n");
            return 0;
        }

    if( !nh.getParam("Kp2",Kp2) )
        {
            ROS_FATAL("Couldn't find parameter: Kp2\n");
            return 0;
        }

    if( !nh.getParam("Kp2",Kv2) )
        {
            ROS_FATAL("Couldn't find parameter: Kv2\n");
            return 0;
        }
    return 1;
}

bool ControlNode::ok()
{
    ros::spinOnce();
    rate.sleep();

    if(!(joint_ok))
        std::cout << "Waiting for incoming messages... did you start the simulation?\n";
    return joint_ok;
}
