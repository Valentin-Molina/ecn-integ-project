//example pgm to set a model state in Gazebo
// could also do w/ rosservice call gazebo/set_model_state
#include <ros/ros.h> //ALWAYS need to include this
//#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "init_model_state");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);
    
    // make sure service is available before attempting to proceed, else node will crash
    bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/set_model_state",true);
      ROS_INFO("waiting for set_model_state service");
      half_sec.sleep();
    }



    ROS_INFO("set_model_state service exists");

    ros::ServiceClient set_model_state_client = 
       nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    double secondes;
    gazebo_msgs::SetModelState model_state_srv_msg;


    //definition parametre cosinus
    double amplitude=0.7;
    double frequence=0.05;



    while (ros::ok()) {
		//hard code, or could prompt, or could have command-line arg here:
		
	secondes=ros::Time::now().toSec();
    model_state_srv_msg.request.model_state.model_name = "cible_simple";
    model_state_srv_msg.request.model_state.pose.position.x = 1.0;
    model_state_srv_msg.request.model_state.pose.position.y = amplitude*cos(2*M_PI*frequence*secondes);
    model_state_srv_msg.request.model_state.pose.position.z = 0.1;
    
    model_state_srv_msg.request.model_state.pose.orientation.x = 0.0;
    model_state_srv_msg.request.model_state.pose.orientation.y = 0.0;
    model_state_srv_msg.request.model_state.pose.orientation.z = 0.0;
    model_state_srv_msg.request.model_state.pose.orientation.w = 0.0;
    
    
    model_state_srv_msg.request.model_state.twist.linear.x= 0.0; //2cm/sec
    model_state_srv_msg.request.model_state.twist.linear.y= 0.0;
    model_state_srv_msg.request.model_state.twist.linear.z= 0.0;
    
    model_state_srv_msg.request.model_state.twist.angular.x= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.y= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.z= 0.0;
        
    model_state_srv_msg.request.model_state.reference_frame = "world";

    set_model_state_client.call(model_state_srv_msg);
        //make sure service call was successful
/*  bool result = model_state_srv_msg.response.success;
    if (result)
        ROS_INFO("Done");
        std::cout << model_state_srv_msg.request.model_state.pose.position.y << std::endl;
*/

    }
    
    
        
}


