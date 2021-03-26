#include "control_node_pid.h"

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_pid_node");
    ControlNodePID controlFinger(1);

    while(ros::ok())
    {
        cout << "-------------" << endl;

        if(controlFinger.ok())
        {
            controlFinger.setTorque();
        }
    }
}
