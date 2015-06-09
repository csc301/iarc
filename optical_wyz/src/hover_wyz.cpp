#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "time.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <opencv2/opencv.hpp>
#include <optical_wyz/optical_wyz_msg.h>
using namespace std;
using namespace cv;
float psf,dsf,pvf;
double out_x,out_y;
geometry_msgs::Point coordinate_sp,velocity_sp;
double e_pre_x,e_pre_y;
geometry_msgs::Twist hover_cmd;
int hover_flag = 0,ps=200,ds=130,pv=300,out_threshold=70,v_max=3;
int coordinate_sp_x =  0,coordinate_sp_y = 0;
ros::Publisher cmd_pub;

void optCallback(const optical_wyz::optical_wyz_msgConstPtr & msg)
{
        geometry_msgs::Point coordinate_m =  msg->coordinate;
        geometry_msgs::Point velocity_m = msg->velocity;
        double e_x,e_y;
        psf = float(ps/100.0);
        dsf = float(ds/100.0);
        //pvf = float(pv/10.0);
        coordinate_sp.x = double(coordinate_sp_x);
        coordinate_sp.y = double(coordinate_sp_y);

        if(hover_flag == 1)
        {
                e_x = coordinate_sp.x - coordinate_m.x;
                e_y = coordinate_sp.y - coordinate_m.y;
                out_x = psf * e_x - dsf * (e_x - e_pre_x);
                out_y = psf * e_y - dsf * (e_y - e_pre_y);
                //if(velocity_sp.x>v_max) velocity_sp.x=v_max;
                //if(velocity_sp.x<-1*v_max) velocity_sp.x=-1*v_max;
                //if(velocity_sp.y>v_max) velocity_sp.y=v_max;
                //if(velocity_sp.y<-1*v_max) velocity_sp.y=-1*v_max;
                //out_x = pvf * (velocity_sp.x - velocity_m.x);
                //out_y = pvf * (velocity_sp.y - velocity_m.y);

                if(out_x>out_threshold) out_x=out_threshold;
                if(out_x<-1*out_threshold) out_x=-1*out_threshold;
                if(out_y>out_threshold) out_y=out_threshold;
                if(out_y<-1*out_threshold) out_y=-1*out_threshold;

                hover_cmd.linear.x = int(out_x);
                hover_cmd.linear.y = int(out_y);
                cmd_pub.publish(hover_cmd);
        }
        else
        {
                coordinate_sp.x=coordinate_m.x;
                coordinate_sp.y=coordinate_m.y;
                hover_cmd.linear.x = 0;
                hover_cmd.linear.y = 0;
                cmd_pub.publish(hover_cmd);
        }
        e_pre_x = e_x;
        e_pre_y = e_y;
        waitKey(1);
}

int main(int argc, char **argv)
{
        ros::init(argc,argv,"hover_wyz_node");
        ros::NodeHandle nh;

        coordinate_sp.x = coordinate_sp.y = 0.0;
        e_pre_x = e_pre_y = 0.0;
        string winname = "hover_wyz_params";
        namedWindow(winname,CV_WINDOW_AUTOSIZE);
        createTrackbar( " hover_flag:", winname, &hover_flag, 1, NULL );
        createTrackbar( " ps:", winname, &ps, 1000, NULL );
        createTrackbar( " ds:", winname, &ds, 1000, NULL );
        //createTrackbar( " pv:", winname, &pv, 1000, NULL );
        createTrackbar( "out_threshold:", winname, &out_threshold, 200, NULL );
        //createTrackbar( "v_max:", winname, &v_max, 6, NULL );
        createTrackbar( "coordinate_sp_x:", winname, &coordinate_sp_x, 5, NULL );
        createTrackbar( "coordinate_sp_y:", winname, &coordinate_sp_y, 5, NULL );
        ros::Subscriber opt_sub = nh.subscribe("optical_wyz",100,optCallback);
        cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",100);
	ros::spin();
}
