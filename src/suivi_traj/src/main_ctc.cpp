#include "control_node.h"

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_ctc");
    ControlNode controlctc;

    if (controlctc.checkParams())
    {
        while(ros::ok())
        {
            cout << "-------------" << endl;

            if(controlctc.ok())
            {
                controlctc.setTorque();
            }
        }
    }
}
