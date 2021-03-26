#include "control_node_pid.h"

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_pid_hand");
    ControlNodePID controlHand(0);

    while(ros::ok())
    {
        cout << "-------------" << endl;

        if(controlHand.ok())
        {
            controlHand.setTorque();
        }
    }
}
