#include "line_detect/points.h"
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Imu.h"
#include <iostream>

using namespace std;

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"pub_points_node");
    ros::NodeHandle nh;
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu_sub",10);
    //ros::Publisher pub_left_point = nh.advertise<line_detect::points>("left_point",10);
    //ros::Publisher pub_right_point = nh.advertise<line_detect::points>("right_point",10);
    //ros::Publisher pub_behinds_point = nh.advertise<line_detect::points>("behind_point",10);
    ros::Rate loop_rate(20);
    //line_detect::points left,right,behind;
    sensor_msgs::Imu imu;
    while(ros::ok())
    {
    /*
        left.point1.x = 1.0;
        left.point1.y = 1.0;
        left.point1.z = -1.0;
        left.point2.x = -1.0;
        left.point2.y = 1.0;
        left.point2.z = -1.0;
        left.havenopoints = false;
        right.point1.x = 1.0;
        right.point1.y = -1.0;
        right.point1.z = -1.0;
        right.point2.x = -1.0;
        right.point2.y = -1.0;
        right.point2.z = -1.0;
        right.havenopoints = false;
        behind.point1.x = -1.0;
        behind.point1.y = 0.0;
        behind.point1.z = -1.0;
        behind.havenopoints = false;
        pub_left_point.publish(left);
        pub_right_point.publish(right);
        pub_behinds_point.publish(behind);
        */
        imu.orientation.x = 0.0;
        imu.orientation.y = 0.0;
        imu.orientation.z = 0.0;
        pub_imu.publish(imu);
        ROS_INFO("publishing points......");
        ros::spinOnce();
        loop_rate.sleep();
    }

}
