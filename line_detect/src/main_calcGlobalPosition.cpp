#include "global.h"
using namespace std;
using namespace cv;

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"calcGlobalPosition_node");
    ros::NodeHandle nh;
	CGlobal MyCGlobal(nh);
    ros::spin();
}
