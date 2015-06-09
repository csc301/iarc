#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include <px_comm/OpticalFlow.h>
#include <px4flow_hover/hover_srv.h>
#include "std_msgs/Bool.h"
#include <math.h>
#include <iostream>
#include <list>
#define MaX  1000
#define SQARE 1.5
using namespace std;
using namespace cv;

bool avo_flag = false;
float sum_x,sum_y,vx,vy,height,height_last,out_z;
float psf,dsf,pvf,vx_ref,vy_ref,sum_x_ref,sum_y_ref,out_x,out_y;
double dt=0,sum_dt=0;
int quality_value,reset=0,point_flag=0,hover_flag=0,given_height=50;
int ps=200,ds=130,pv=300,out_threshold=70,v_max=3,hp = 60,hd = 40;
ros::Time  t_now,t_last;
ros::Publisher path_publisher,point_publisher,hover_publisher;
ros::ServiceServer hover_srv;
nav_msgs::Path path_msg;
geometry_msgs::PoseStamped pose_msg;
geometry_msgs::PointStamped point_msg;
geometry_msgs::Twist hover_cmd;
list<float> fliter_vx(5,0),fliter_vy(5,0);

void opt_flow_callback(const px_comm::OpticalFlow::ConstPtr& msg )
{
	height = msg -> ground_distance ;          
	vx = msg -> velocity_x;                      
	vy = -1.0*msg -> velocity_y;                      	
	quality_value = msg ->quality;  
       
       float sum_vx=0,sum_vy=0;
       fliter_vx.push_front(vx);
       fliter_vy.push_front(vy);
       fliter_vx.pop_back();
       fliter_vy.pop_back();
       for(list<float>::iterator vx_iter=fliter_vx.begin();vx_iter!=fliter_vx.end();vx_iter++)sum_vx+=*vx_iter;
       vx = sum_vx/fliter_vx.size();
       for(list<float>::iterator vy_iter=fliter_vy.begin();vy_iter!=fliter_vy.end();vy_iter++)sum_vy+=*vy_iter;
       vy = sum_vy/fliter_vy.size();

       t_now = msg->header.stamp;
       dt = (double( t_now.sec + 1e-9 * t_now.nsec )) - (double( t_last.sec + 1e-9 * t_last.nsec ));
       t_last = t_now;
       sum_x += vx * dt;
       sum_y += vy * dt;
       if(reset == 1)
       {
              sum_x = 0;
              sum_y = 0;
              sum_x_ref=0;
              sum_y_ref=0;
              sum_dt=0;
       }

       psf = float(ps/100.0);
       dsf = float(ds/100.0);
       pvf = float(pv/10.0);
       
       if(hover_flag == 1)
       {
            vx_ref = psf * (sum_x_ref - sum_x) - dsf*vx;
            vy_ref = psf * (sum_y_ref - sum_y) - dsf*vy;
            if(vx_ref>v_max)vx_ref=v_max;
            if(vx_ref<-1*v_max)vx_ref=-1*v_max;
            if(vy_ref>v_max)vy_ref=v_max;
            if(vy_ref<-1*v_max)vy_ref=-1*v_max;
            out_x = pvf * (vx_ref - vx);
            out_y = pvf * (vy_ref - vy);
 
            if(out_x>out_threshold)out_x=out_threshold;
            if(out_x<-1*out_threshold)out_x=-1*out_threshold;
            if(out_y>out_threshold)out_y=out_threshold;
            if(out_y<-1*out_threshold)out_y=-1*out_threshold;
            hover_cmd.linear.x = int(out_x);
            hover_cmd.linear.y = int(out_y);

            out_z = hp*((given_height+100.0)*0.01-height)-hd*(height-height_last);
            height_last = height;
            if(out_z>50)out_z=50;
            if(out_z<-50)out_z=-50;
            if(out_z<0)out_z-=15;
            hover_cmd.linear.z = int(out_z);
            hover_publisher.publish(hover_cmd);          
       }
       else
       {
              sum_dt=0;
              sum_x_ref=sum_x;
              sum_y_ref=sum_y;
              hover_cmd.linear.x = 0;
              hover_cmd.linear.y = 0;

             out_z = hp*((given_height+100.0)*0.01-height)-hd*(height-height_last);
             height_last = height;
             if(out_z>50)out_z=50;
             if(out_z<-50)out_z=-50;
             if(out_z<0)out_z-=15;
             hover_cmd.linear.z = int(out_z);
             hover_publisher.publish(hover_cmd);
             //cout << out_z <<endl;

       }

       if(avo_flag == true)
       { 
              sum_x_ref=sum_x;
              sum_y_ref=sum_y;
       }

       point_msg.header.frame_id = "my_frame";

       point_msg.point.x = sum_x;
       point_msg.point.y = sum_y;
       point_msg.point.z = height;

       point_flag++;
       if(point_flag == 15)
       {
             point_publisher.publish(point_msg);
             point_flag=0;
       }
	        
       waitKey(1); 
}

void avo_flag_callback(const std_msgs::Bool avo_flag_msg)
{
   avo_flag =  avo_flag_msg.data;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"flow_position_node");
    ros::NodeHandle n;

    ros::Subscriber px4flow_sub = n.subscribe("/px4flow/opt_flow", 100, opt_flow_callback);
    ros::Subscriber avo_flag_sub = n.subscribe("/obstacle_avoid_flag", 100, avo_flag_callback);

    //path_publisher = n.advertise<nav_msgs::Path>("uav_path",100);
    point_publisher = n.advertise<geometry_msgs::PointStamped>("uav_point",100);
    hover_publisher = n.advertise<geometry_msgs::Twist>("hover_cmd",100);

    namedWindow("uav_path_parameter_tuning",CV_WINDOW_AUTOSIZE);
    moveWindow("uav_path_parameter_tuning",240,180);
    createTrackbar( " reset:", "uav_path_parameter_tuning", &reset, 1, NULL );
    createTrackbar( " hover:", "uav_path_parameter_tuning", &hover_flag, 1, NULL );
    createTrackbar( " ps:", "uav_path_parameter_tuning", &ps, MaX, NULL );
    createTrackbar( " ds:", "uav_path_parameter_tuning", &ds, MaX, NULL );
    createTrackbar( " pv:", "uav_path_parameter_tuning", &pv, MaX, NULL );
    createTrackbar( "threshold:", "uav_path_parameter_tuning", &out_threshold, 100, NULL );
    createTrackbar( "v_max:", "uav_path_parameter_tuning", &v_max, 6, NULL );
    createTrackbar( "given_height:", "uav_path_parameter_tuning", &given_height, 100, NULL );
    createTrackbar( "hp:", "uav_path_parameter_tuning", &hp, 150, NULL );
    createTrackbar( "hd:", "uav_path_parameter_tuning", &hd, 100, NULL );
    ros::spin();
    return 0;
}