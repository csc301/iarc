#include "multi_object_.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"multi_object_node");
    ros::NodeHandle nh;
    multi_object_ multi_obj(nh);
    return 0;
}
