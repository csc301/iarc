#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <px4flow_hover/imu_data_srv.h>
#include <px_comm/OpticalFlow.h>
#include <math.h>
#include <list>
#define G 9.8
#define MaX  400
using namespace std;
using namespace cv;

float vx,vy,ax,ay,height,ax_ref,ay_ref,x_out,y_out,pvf,dvf,paf,  ix,iy,ivf;
int quality_value,ref_threshold=80,out_threshold=80,pv=2000,dv=120,pa=80,iv=30;
px4flow_hover::imu_data_srv  srv;
ros::ServiceClient imu_client ;
sensor_msgs::Imu imu_msg;
ros::Publisher  hover_publisher,flow_pub;
geometry_msgs::Twist hover_msg,zero,flow;
int flag = 0 ; 

list<float> px4_vx(10,0),px4_vy(10,0);
list<float> out_vx(10,0),out_vy(10,0);

//list<float>::iterator vx_iter,vy_iter;


void opt_flow_callback(const px_comm::OpticalFlow::ConstPtr& msg )
{
         float sum_vx=0,sum_vy=0;
	//height = msg -> ground_distance ;          
	vx = msg -> velocity_x;                      
	vy = -1.0*msg -> velocity_y;                      	
	//quality_value = msg ->quality;  

        
        px4_vx.push_front(vx);
        px4_vy.push_front(vy);
        px4_vx.pop_back();
        px4_vy.pop_back();

        for(list<float>::iterator vx_iter=px4_vx.begin();vx_iter!=px4_vx.end();vx_iter++)sum_vx+=*vx_iter;
        
        vx = sum_vx/px4_vx.size();

        for(list<float>::iterator vy_iter=px4_vy.begin();vy_iter!=px4_vy.end();vy_iter++)sum_vy+=*vy_iter;
        vy = sum_vy/px4_vy.size();   
        sum_vx=0;
        sum_vy=0;

        flow.linear.x = vx;
        flow.linear.y = vy;
        flow_pub.publish(flow);
        
	//cout<<px4_vx.size()<<endl;

           //printf("%f,%f\n",vx,vy );

	if (imu_client.call(srv))
	{
		imu_msg = srv.response.imu_data_;
		ax = G * tan(imu_msg.orientation.x);    //pitch
		ay = G * tan(imu_msg.orientation.y);    //roll
		flag = 1;
                //printf("%f,%f\n",ax,ay );
	}
	else
	{
		ROS_ERROR("Failed to call imu_service");
	}
	if(flag == 1)
	{
                        flag = 0;
		        pvf = (float)(pv/10.0);
		        dvf = (float)(dv/10.0);
		        paf = (float)(pa/8.0);
                        ivf = float (iv/100.0);
                        //printf("%f,%f,%f\n",pvf,dvf,paf);
                        if(ix<100&&ix>-100)ix+=vx;
                        if(iy<100&&iy>-100)iy+=vy;
                        

         		ax_ref = -vx*pvf - ax*dvf - ivf*ix;
         		ay_ref = -vy*pvf - ay*dvf - ivf*iy;

         		if(ax_ref>ref_threshold)ax_ref = ref_threshold;
         		else if(ax_ref<(-1)*ref_threshold)ax_ref = (-1)*ref_threshold;
   		        if(ay_ref>ref_threshold)ay_ref = ref_threshold;
         		else if(ay_ref<(-1)*ref_threshold)ay_ref = (-1)*ref_threshold;

         		//x_out = paf*(ax_ref - ax);
         		//y_out = paf*(ay_ref - ay);

                        /*if(x_out>1.0)x_out = 1.0;
         		if(x_out<-1.0)x_out = -1.0;
         		if(y_out>1.0)y_out = 1.0;
         		if(y_out<-1.0)y_out = -1.0;
                      
                        if(x_out>0)x_out=x_out*50+0;
                        else if (x_out<0)x_out=x_out*50-0;
                        if(y_out>0)y_out=y_out*50+0;
                        else if (y_out<0)y_out=y_out*50-0;
                        */

         		/*if(x_out>out_threshold)x_out = out_threshold;
         		if(x_out<(-1)*out_threshold)x_out = (-1)*out_threshold;
         		if(y_out>out_threshold)y_out = out_threshold;
         		if(y_out<(-1)*out_threshold)y_out = (-1)*out_threshold;
                         */

        		out_vx.push_front(ax_ref);
       			out_vy.push_front(ay_ref);
        		out_vx.pop_back();
        		out_vy.pop_back();

                        for(list<float>::iterator vx_iter=out_vx.begin();vx_iter!=out_vx.end();vx_iter++)sum_vx+=*vx_iter;
        		ax_ref = sum_vx/out_vx.size();

        		for(list<float>::iterator vy_iter=out_vy.begin();vy_iter!=out_vy.end();vy_iter++)sum_vy+=*vy_iter;
        		ay_ref = sum_vy/out_vy.size();  
                        sum_vx=0;
                        sum_vy=0;
                        
                     
		        hover_msg.linear.x = int(ax_ref);
             		hover_msg.linear.y = int(ay_ref);
             		hover_publisher.publish(hover_msg);
	}        
        else  hover_publisher.publish(zero);
          
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"hover_controller_node");
    ros::NodeHandle n;
    zero.linear.x = 0;
    zero.linear.y = 0;
    zero.linear.z = 0;
    zero.angular.x = 0;
    zero.angular.y = 0;
    zero.angular.z = 0;

    ros::Subscriber px4flow_sub = n.subscribe("/px4flow/opt_flow", 100, opt_flow_callback);
    imu_client = n.serviceClient<px4flow_hover::imu_data_srv>("imu_data_srv");
    hover_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",100);
    flow_pub = n.advertise<geometry_msgs::Twist>("flow_vel",100);

    namedWindow("hover_controller_parameter_tuning",CV_WINDOW_AUTOSIZE);
    createTrackbar( " pv:", "hover_controller_parameter_tuning", &pv, MaX*10, NULL );
    createTrackbar( " dv:", "hover_controller_parameter_tuning", &dv, MaX, NULL );
    createTrackbar( " iv:", "hover_controller_parameter_tuning", &iv, 100, NULL );
    createTrackbar( " ref_threshold:", "hover_controller_parameter_tuning", &ref_threshold,100, NULL );
    //createTrackbar( " out_threshold:", "hover_controller_parameter_tuning", &out_threshold,100, NULL );
    ros::Rate loop_rate(250);
      while(ros::ok())
      {
             waitKey(30);
             ros::spinOnce();
             loop_rate.sleep();
      }
    return 0;
}
