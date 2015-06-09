#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

using namespace std;

geometry_msgs::Twist hover_msg,avo_msg;
ros::Publisher decision_publisher;

void hover_cmd_callback(const geometry_msgs::Twist msgforhover)
{
  hover_msg=msgforhover;
}

void avo_cmd_callback(const geometry_msgs::Twist msgforavo)
{
  avo_msg =msgforavo;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"decision_node");
    ros::NodeHandle n;

    decision_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",100);
    ros::Subscriber hover_cmd_sub = n.subscribe("/hover_cmd", 100, hover_cmd_callback);
    ros::Subscriber avo_cmd_sub = n.subscribe("/avo_cmd", 100, avo_cmd_callback);

    ros::Rate loop_rate(250);
    while(ros::ok())
    {
       ros::spinOnce();
      
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
