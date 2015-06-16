#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "geometry_msgs/Twist.h"
#include <px_comm/OpticalFlow.h>
#include <iostream>
#define FOCUS 121
#define out_threshold  70
using namespace std;

geometry_msgs::Twist tracker_cmd;
ros::Publisher tracker_publisher;
float height;
float vx,vy,error_x,error_y,out_x,out_y;

void height_callback(const px_comm::OpticalFlow::ConstPtr& msg)
{
      height = msg -> ground_distance;
      vx = msg -> velocity_x;                      
      vy = -1.0*msg -> velocity_y; 
}

void roi_callback(const sensor_msgs::RegionOfInterest::ConstPtr& msg)
{
           error_x = (height/FOCUS)*(240-(msg->y_offset+0.5*msg->height))-0.1;
           error_y = (height/FOCUS)*(360-(msg->x_offset+0.5*msg->width));

            out_x = 30 * (20 *error_x- 13*vx);
            out_y = 30 * (20 *error_y - 13*vy);
 
            cout << error_x <<" "<< error_y << "   "<< out_x << out_y  <<endl;

            if(out_x>out_threshold)out_x=out_threshold;
            if(out_x<-1*out_threshold)out_x=-1*out_threshold;
            if(out_y>out_threshold)out_y=out_threshold;
            if(out_y<-1*out_threshold)out_y=-1*out_threshold;
            tracker_cmd.linear.x = int(out_x);
            tracker_cmd.linear.y = int(out_y);

            tracker_cmd.angular.x = true;
            tracker_publisher.publish(tracker_cmd);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"multi_tracker_node");
	ros::NodeHandle n;
	ros::Subscriber  height_sub = n.subscribe("/px4flow/opt_flow",10,height_callback);
             ros::Subscriber  roi_sub = n.subscribe("/sigle_roi",10,roi_callback);
             tracker_publisher = n.advertise<geometry_msgs::Twist>("tracker_cmd",10);

             ros::Rate loop_rate(250);
             while(ros::ok())
             {
                 ros::spinOnce();      
                 loop_rate.sleep();
              }
	return 0;
}
