#include "cmd_vel_serial.h"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"cmd_vel_serial_node");
    ros::NodeHandle n;     

    cmd_vel_serial cmd_vel_serial_obj(n);

    return 0;
}

