#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <px_comm/OpticalFlow.h>
#include <iostream>

using namespace std;
using namespace cv;

geometry_msgs::Twist hover_msg,avo_msg,tracker_msg;
ros::Publisher decision_publisher;
float out_z,height,height_last;
double hei_error=0,sum_err=0;
int given_height=100,hp =80,hd = 60,hi=100,removei=50,haha=5;

int avo_flag=0,tracker_flag=0;

void hover_cmd_callback(const geometry_msgs::Twist msgforhover)
{
  hover_msg=msgforhover;
}

void avo_cmd_callback(const geometry_msgs::Twist msgforavo)
{
  avo_msg =msgforavo;
  avo_flag=0;
}

void tracker_cmd_callback(const geometry_msgs::Twist msgfortracker)
{
  tracker_msg =msgfortracker;
  tracker_flag=0;
}

void height_callback(const px_comm::OpticalFlow::ConstPtr& msg)
{
      height = msg -> ground_distance;
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"decision_node");
    ros::NodeHandle n;

    decision_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",100);
    ros::Subscriber hover_cmd_sub = n.subscribe("/hover_cmd", 100, hover_cmd_callback);
    ros::Subscriber avo_cmd_sub = n.subscribe("/avo_cmd", 100, avo_cmd_callback);
    ros::Subscriber tracker_cmd_sub = n.subscribe("/tracker_cmd", 100, tracker_cmd_callback);
    ros::Subscriber  height_sub = n.subscribe("/px4flow/opt_flow",10,height_callback);

    namedWindow( "height_parameter_tuning",WINDOW_NORMAL);
    createTrackbar( "given_height:", "height_parameter_tuning", &given_height, 150, NULL );
    createTrackbar( "hp:", "height_parameter_tuning", &hp, 150, NULL );
    createTrackbar( "hd:", "height_parameter_tuning", &hd, 100, NULL );
    createTrackbar( "hi:", "height_parameter_tuning", &hi, 1000, NULL );
    createTrackbar( "removei:", "height_parameter_tuning", &removei, 100, NULL );
    createTrackbar( "haha:", "height_parameter_tuning", &haha, 15, NULL );

    ros::Rate loop_rate(250);
    while(ros::ok())
    {
       hei_error = (given_height+50.0)*0.01-height;
       if (hei_error<removei/100.0 && hei_error>removei/(-100.0) && out_z<55 && out_z>-75)
       {
              sum_err+=(hei_error/haha);
       }

       if (sum_err*hei_error<0)
       {
         sum_err=0;
       }

       out_z = hp*hei_error-hd*(height-height_last) + (hi/100.0)*sum_err;

       height_last = height;
       if(out_z>55)out_z=55;
       if(out_z<-55)out_z=-55;
       if(out_z<0)out_z-=20;

       avo_msg.linear.z = int(out_z);
       tracker_msg.linear.z = int(out_z);
       hover_msg.linear.z = int(out_z);
      
       avo_flag++;
       if (avo_flag>150)
       {
         avo_flag=0;
         avo_msg.angular.x=0;
       }

       tracker_flag++;
       if (tracker_flag>150)
       {
         tracker_flag=0;
         tracker_msg.angular.x=0;
       }

        if(avo_msg.angular.x==1)
        {
          decision_publisher.publish(avo_msg);
         //cout<<"avo"<< endl;
        }
        else if (tracker_msg.angular.x==1)
        {
           decision_publisher.publish(tracker_msg);
           //cout<<"tracker"<< endl;        
        }
        else 
        {
           decision_publisher.publish(hover_msg);
          // cout<<"hover"<< endl;        
        }

       ros::spinOnce();
       loop_rate.sleep();
       waitKey(1); 
    }
    return 0;
}
