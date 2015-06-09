#include <iostream>
#include "ros/ros.h"
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"image_pub_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher img_pub0 = it.advertise("image_raw",1);
    VideoCapture cap0(0);
  Mat frame0;
    //image_transport::Publisher img_pub1 = it.advertise("camera/image_raw1",1);
    sensor_msgs::ImagePtr msg0,msg1;
    ros::Rate loop_rate(20);
    while(nh.ok())
    {
        cap0.read(frame0);
        //cap1.read(frame1);
        if(frame0.empty())cout<<"frame0 is empty"<<endl;
        //if(frame1.empty())cout<<"frame1 is empty"<<endl;
        msg0=cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame0).toImageMsg();
       //msg1=cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame1).toImageMsg();
        img_pub0.publish(msg0);
        //img_pub1.publish(msg1);
        loop_rate.sleep();
        ros::spinOnce();
    }

}
