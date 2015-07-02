#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/RegionOfInterest.h"
#include "geometry_msgs/Twist.h"
#include <px_comm/OpticalFlow.h>
#include <iostream>
#define FOCUS 100
#define MaX  1000
#define out_threshold 80
using namespace std;
using namespace cv;

std_msgs::Bool tracker_flag;
geometry_msgs::Twist tracker_cmd;
ros::Publisher tracker_publisher,tracker_flag_pub;
float height=0;
float vx,vy,error_x,error_y,out_x,out_y;
int tracker_counter=0;
int ps=310,ds=280,pv=300;
float psf,dsf,pvf;

void height_callback(const px_comm::OpticalFlow::ConstPtr& msg)
{
      height = msg -> ground_distance;
      vx = msg -> velocity_x;                      
      vy = -1.0*msg -> velocity_y; 
}

void roi_callback(const sensor_msgs::RegionOfInterest::ConstPtr& msg)
{
           error_x = (height/FOCUS)*(120-(msg->y_offset+0.5*msg->height));
           error_y = (height/FOCUS)*(160-(msg->x_offset+0.5*msg->width));

             psf = float(ps/100.0);
             dsf = float(ds/100.0);
             pvf = float(pv/10.0);

            out_x = pvf  * ( psf *error_x-dsf*vx);
            out_y = pvf  * ((psf+0.5)*error_y - dsf*vy);
 
            //cout <<int(msg->x_offset) <<" "<<int(msg->y_offset)<<endl;
            cout << error_x <<" "<< error_y << "   "<< out_x <<" "<<  out_y  <<endl;

            if(out_x>out_threshold)out_x=out_threshold;
            if(out_x<-1*out_threshold)out_x=-1*out_threshold;
            if(out_y>out_threshold)out_y=out_threshold;
            if(out_y<-1*out_threshold)out_y=-1*out_threshold;
            tracker_cmd.linear.x = int(out_x);
            tracker_cmd.linear.y = int(out_y);

            tracker_cmd.angular.x = true;
            tracker_publisher.publish(tracker_cmd);
            tracker_flag.data=true;
            tracker_flag_pub.publish(tracker_flag);
            tracker_counter=0;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"multi_tracker_node");
	ros::NodeHandle n;
	ros::Subscriber  height_sub = n.subscribe("/px4flow/opt_flow",10,height_callback);
       ros::Subscriber  roi_sub = n.subscribe("/sigle_roi",10,roi_callback);
       tracker_publisher = n.advertise<geometry_msgs::Twist>("tracker_cmd",10);
       tracker_flag_pub = n.advertise<std_msgs::Bool>("irobot_tracker_flag",10);

    namedWindow("tracker_parameter_tuning",WINDOW_NORMAL);  //WINDOW_NORMAL   CV_WINDOW_AUTOSIZE
    moveWindow("tracker_parameter_tuning",240,180);
    createTrackbar( " ps:", "tracker_parameter_tuning", &ps, MaX, NULL );
    createTrackbar( " ds:", "tracker_parameter_tuning", &ds, MaX, NULL );
    createTrackbar( " pv:", "tracker_parameter_tuning", &pv, MaX, NULL );

             ros::Rate loop_rate(250);
             while(ros::ok())
             {
                 tracker_counter++;
                 if (tracker_counter>150)
                 {
                    tracker_counter=0;
                    tracker_flag.data=false;
                    tracker_flag_pub.publish(tracker_flag);
                    tracker_cmd.angular.x = false;
                    tracker_publisher.publish(tracker_cmd);
                 }

                 ros::spinOnce();      
                 loop_rate.sleep();
                 waitKey(1); 
              }
	return 0;
}
