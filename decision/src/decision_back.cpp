#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <px4flow_hover/hover_srv.h>
#include "simple_obstacle_avoidance/avo_srv.h"
#include <iostream>

using namespace std;

ros::ServiceClient hover_client,avo_client;
geometry_msgs::Twist hover_msg,avo_msg;
ros::Publisher decision_publisher;
px4flow_hover::hover_srv srv_hover;
simple_obstacle_avoidance::avo_srv srv_avo;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"decision_node");
    ros::NodeHandle n;
    hover_client = n.serviceClient<px4flow_hover::hover_srv>("hover_srv");
    avo_client = n.serviceClient<simple_obstacle_avoidance::avo_srv>("avo_srv");
    decision_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",100);

    ros::Rate loop_rate(150);
    while(ros::ok())
    {
       ros::spinOnce();
       if (hover_client.call(srv_hover))
       {
	    hover_msg = srv_hover.response.hover_srv;
            //cout <<hover_msg.linear.x<<" "<<hover_msg.linear.y<<endl;
	}
	else
	{
            ROS_ERROR("Failed to call hover_service");
	}
 
        if (avo_client.call(srv_avo))
        {
	    avo_msg = srv_avo.response.avo_action;
            //cout <<avo_msg.linear.x<<" "<<avo_msg.linear.y<<endl;
	}
	else
	{
            ROS_ERROR("Failed to call avo_service");
	}
      
        if(avo_msg.angular.x==1)
        {
          decision_publisher.publish(avo_msg);
         // cout<<"avo"<< endl;
        }
        else 
        {
           decision_publisher.publish(hover_msg);
          // cout<<"hover"<< endl;        
        }
       loop_rate.sleep();
    }
    return 0;
}
