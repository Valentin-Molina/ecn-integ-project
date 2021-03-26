#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <math.h>
#include <sstream>
#include <iostream>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
//inutile mais peut servir pour creer nos propres messages
//#include <Suivi_traj_EK/Commande.h>


using namespace std;

// global variables for subscriber
sensor_msgs::JointState etat;
sensor_msgs::JointState traj;
std_msgs::Float64 commandeq1, commandeq2;


void etatCallback(const sensor_msgs::JointStatePtr & msg)
{
    etat = *msg;
}

void trajCallback(const sensor_msgs::JointStatePtr & msg)
{
    traj = *msg;
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "PID_controller");
    ros::NodeHandle nh;

    // subscriber Etat (fourni par le modele du grp 1)
    ros::Subscriber etat_sub = nh.subscribe ("/joint_states", 10, etatCallback);

    // subscriber Trajectoire (fourni par le grp 4)
    ros::Subscriber traj_sub = nh.subscribe ("/Trajectoire", 10, trajCallback);

    // publisher effort q1
    ros::Publisher couple1_pub = nh.advertise<std_msgs::Float64>("/hand_effort_controller/command", 10);

    // publisher effort q2
    ros::Publisher couple2_pub = nh.advertise<std_msgs::Float64>("/finger_effort_controller/command", 10);

    float Kp0=120, Kp1=100, Kd0=0.1, Kd1=0.1, Ki0=0.1, Ki1=0.1, Te=0.01;
    float integrale[2] = {};
    float err[2] = {}, err_p[2] = {};


    ros::Rate rate(1/Te);

	etat.position.resize(2);
    etat.velocity.resize(2);

    traj.position.resize(2);
    traj.velocity.resize(2);





    while (ros::ok())
    {

        err[0] = traj.position[0] - etat.position[0];
        err_p[0] = traj.velocity[0] - etat.velocity[0];
        integrale[0] += err[0]*Te;
        commandeq1.data = Kp0*err[0] + Kd0*err_p[0]/Te + Ki0*integrale[0];

        //cout<<"commande P: "<<Kp0*err[0]<<endl;
        //cout<<"commande D: "<<Kd0*err_p[0]/Te<<endl;
        //cout<<"commande I: "<<Ki0*integrale[0]<<endl;


        err[1] = traj.position[1] - etat.position[1];
        err_p[1] = traj.velocity[1] - etat.velocity[1];
        integrale[1] += err[1]*Te;
        commandeq2.data = Kp1*err[1] + Kd1*err_p[1]/Te + Ki1*integrale[1];


        //cout<<"commande P: "<<Kp1*err[1]<<endl;
        //cout<<"commande D: "<<Kd1*err_p[1]/Te<<endl;
        //cout<<"commande I: "<<Ki1*integrale[1]<<endl;


        //cout<<"etat position q1: "<<etat.position[0]<<endl;
        //cout<<"etat position q2: "<<etat.position[1]<<endl;

        cout<<"error position q1: "<<err[0]<<endl;
        cout<<"error position q2: "<<err[1]<<endl;
        
        
        // publish setpoint
        couple1_pub.publish(commandeq1);
        couple2_pub.publish(commandeq2);

        ros::spinOnce();
        rate.sleep();
    }

}
