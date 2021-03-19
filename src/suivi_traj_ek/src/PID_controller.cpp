#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <math.h>
#include <sstream>
#include <iostream>

#include <sensor_msgs/JointState.h>
//inutile mais peut servir pour creer nos propres messages
//#include <Suivi_traj_EK/Commande.h>


using namespace std;

// global variables for subscriber
sensor_msgs::JointState etat;
sensor_msgs::JointState traj;
sensor_msgs::JointState commande;


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
    ros::Subscriber etat_sub = nh.subscribe ("/EtatRobot", 10, etatCallback);

    // subscriber Trajectoire (fourni par le grp 4)
    ros::Subscriber traj_sub = nh.subscribe ("/Trajectoire", 10, trajCallback);

    // publisher
    ros::Publisher couple_pub = nh.advertise<sensor_msgs::JointState>("/CommandeMoteur", 10);

    float Kp0=1, Kp1=1, Kd0=1, Kd1=1, Ki0=1, Ki1=1, Te=0.01;
    float integrale[2] = {};
    float err[2] = {}, err_p[2] = {};


    ros::Rate rate(1/Te);

	etat.position.resize(2);
    etat.velocity.resize(2);

    traj.position.resize(2);
    traj.velocity.resize(2);



    commande.name.resize(2);
    commande.effort.resize(2);


    while (ros::ok())
    {

        err[0] = traj.position[0] - etat.position[0];
        err_p[0] = traj.velocity[0] - etat.velocity[0];
        integrale[0] += err[0]*Te;
        commande.effort[0] = Kp0*err[0] + Kd0*err_p[0] + Ki0*integrale[0];
        commande.name[0] = "q1";

        err[1] = traj.position[1] - etat.position[1];
        err_p[1] = traj.velocity[1] - etat.velocity[1];
        integrale[1] += err[1]*Te;
        commande.effort[1] = Kp1*err[1] + Kd1*err_p[1] + Ki1*integrale[1];
        commande.name[1] = "q2";




        cout<<"error position q1: "<<err[0]<<endl;
        cout<<"error position q2: "<<err[1]<<endl;
        
        
        // publish setpoint
        couple_pub.publish(commande);

        ros::spinOnce();
        rate.sleep();
    }

}
