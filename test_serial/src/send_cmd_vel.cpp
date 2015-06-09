#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/Twist.h"
#define max   600

using namespace cv;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"send_cmd_vel_node");
    ros::NodeHandle n;     
    ros::Rate loop_rate(30);
    geometry_msgs::Twist msg;
    ros::Publisher  cmd_vel_publisher;
    cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",100);
    int roll=300,pitch=300,yaw=300,throttle=300;

    namedWindow("test_serial",CV_WINDOW_AUTOSIZE);
    createTrackbar( " roll:", "test_serial", &roll, max, NULL );
    createTrackbar( " pitch:", "test_serial", &pitch, max, NULL );
    createTrackbar( " yaw:", "test_serial", &yaw, max, NULL );
    createTrackbar( " throttle:", "test_serial", &throttle, max, NULL );

      while(ros::ok())
      {
             waitKey(30);
             msg.linear.x = pitch-300;
             msg.linear.y = roll-300;
             msg.linear.z = throttle-300;
             msg.angular.z = yaw-300;

              cmd_vel_publisher.publish(msg);
      	       ros::spinOnce();
              loop_rate.sleep();
      }
    return 0;
}

