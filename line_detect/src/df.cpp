//#include "linefinder.h"
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include <fstream>

using namespace std;
#define FILTER_SIZE 5
double filter[FILTER_SIZE] = {1,3,3,3,3};
double sum_filter = 13;
double xBufferForFilter[FILTER_SIZE] = {0};
double yBufferForFilter[FILTER_SIZE] = {0};
ofstream file1("/home/ycc/iarc/src/line_detect/df_coordinate.txt");


geometry_msgs::Point df_point;

    void coordinateCallback(const geometry_msgs::PointConstPtr& msg)
    {
        double x = msg->x;
        double y = msg->y;
        double dfx,dfy;
        //if(abs(x)>MAX_)
           // x = xBufferForFilter[FILTER_SIZE-1];
       // if(abs(y)>MAX_)
            //y = yBufferForFilter[FILTER_SIZE-1];
        for (int i=0;i!=FILTER_SIZE;i++)
        {
            xBufferForFilter[i] = xBufferForFilter[i+1];
            yBufferForFilter[i] = yBufferForFilter[i+1];
        }
        xBufferForFilter[FILTER_SIZE-1] = x;
        yBufferForFilter[FILTER_SIZE-1] = y;
        for(int i=0;i!=FILTER_SIZE;i++)
        {
            dfx += xBufferForFilter[i] * filter[i];
            dfy += yBufferForFilter[i] * filter[i];
        }
        dfx = dfx / sum_filter;
        dfy = dfy / sum_filter;
        //if(abs(x)<minvalue)Rtn.x = 0;
        //if(abs(y)<minvalue)Rtn.y = 0;
        xBufferForFilter[FILTER_SIZE-1] = dfx;
        yBufferForFilter[FILTER_SIZE-1] = dfy;

        df_point.x = dfx;
        df_point.y = dfy;
        df_point.z = msg->z;
        file1<<dfx<<" "<<dfy<<" "<<msg->z<<endl;

    }

int main(int argc,char ** argv)
{
	ros::init(argc,argv,"df_node");
	ros::NodeHandle nh;
	ros::Publisher pub_df_coordinate = nh.advertise<geometry_msgs::Point>("df_coordinate",10);
	ros::Subscriber sub_coordinate = nh.subscribe("coordinate",10,coordinateCallback);
	ros::Rate loop_rate(20);
	while(ros::ok())
	{
        pub_df_coordinate.publish(df_point);
        ros::spinOnce();
        loop_rate.sleep();
	}
}
