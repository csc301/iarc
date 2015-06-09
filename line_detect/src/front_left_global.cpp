#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "linefinder.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
using namespace std;
using namespace cv;

double Height;
double Px,Py;
bool noFront,noLeft;
Mat R_b2w;
double pitch0,roll0,yaw0;

void heightCallback(const std_msgs::Float32ConstPtr& msg)
{
    Height = msg->data;
    //cout<<"Height= "<<Height<<endl;
    //ROS_INFO("height= %f",Height);
};

void frontCallback(const line_detect::pointsConstPtr& msg)
{
    noFront = msg->havenopoints;
    //ROS_INFO("FRONT");
    if(!noFront)
    {
        //cout<<msg->point1.x<< " " <<msg->point1.y<<" "<<msg->point1.z<<endl;
	Vec3d point;
	Mat pointw = Mat(3,1,CV_64FC1);
	point[0] = msg->point1.x;
	point[1] = msg->point1.y;
	point[2] = msg->point1.z;
	pointw = R_b2w * Mat(point);
        //Px = 4.5 + Height*(msg->point1.x / msg->point1.z);
	Px = 4.5 + Height*(pointw.at<double>(0,0) / pointw.at<double>(0,2));
    }
};

void leftCallback(const line_detect::pointsConstPtr& msg)
{
//ROS_INFO("LEFT");
    noLeft = msg->havenopoints;
    if(!noLeft)
    {
        //cout<<msg->point1.x<< " " <<msg->point1.y<<" "<<msg->point1.z<<endl;
        //Py = 4.5 + Height*(msg->point1.y / msg->point1.z);
	Vec3d point;
	Mat pointw = Mat(3,1,CV_64FC1);
	point[0] = msg->point1.x;
	point[1] = msg->point1.y;
	point[2] = msg->point1.z;
	pointw = R_b2w * Mat(point);
	//cout<<R_b2w<<endl;
	Px = 4.5 + Height*(pointw.at<double>(0,1) / pointw.at<double>(0,2));
    }
};

void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
	double roll = msg->orientation.x - roll0;
	double pitch = msg->orientation.y - pitch0;
	//double yaw = msg->orientation.z - yaw0;
	double yaw = 0.0;
	//cout<<"pitch= "<<pitch<<" roll= "<<roll<<" yaw= "<<yaw<<endl;
	double rz[]={cos(yaw),sin(yaw),0,
		-sin(yaw),cos(yaw),0,
		0,0,1};
	double ry[]={cos(roll),0,-sin(roll),
		0,1,0,
		sin(roll),0,cos(roll)};
	double rx[]={1,0,0,
		0,cos(pitch),sin(pitch),
		0,-sin(pitch),cos(pitch)};
	Mat Rz = Mat(3,3,CV_64FC1);
	Mat Ry = Mat(3,3,CV_64FC1);
	Mat Rx = Mat(3,3,CV_64FC1);
	Mat R_w2b = Mat(3,3,CV_64FC1);
	//R_b2w = Mat(3,3,CV_64FC1);

	for(int i=0;i!=Rz.rows;i++)
		for (int j=0;j!=Rz.cols;j++)
			Rz.at<double>(i,j) = rz[i*Rz.cols+j];
	for(int i=0;i!=Ry.rows;i++)
		for (int j=0;j!=Ry.cols;j++)
			Ry.at<double>(i,j) = ry[i*Ry.cols+j];
	for(int i=0;i!=Rx.rows;i++)
		for (int j=0;j!=Rx.cols;j++)
			Rx.at<double>(i,j) = rx[i*Rx.cols+j];
	R_w2b = Rx*Ry*Rz;
	R_b2w = R_w2b.inv();
};


int main(int argc,char ** argv)
{
	ros::init(argc,argv,"front_left_global_node");
	ros::NodeHandle nh;
	Px = Py = Height = 0.0;
	if(!nh.getParam("pitch0",pitch0)) pitch0 = 0;
	if(!nh.getParam("roll0",roll0)) roll0 = 0;
	if(!nh.getParam("yaw0",yaw0)) yaw0 = 0;
	ros::Subscriber height_sub = nh.subscribe("ultrasonic_data",30,heightCallback);
	ros::Subscriber front_sub = nh.subscribe("front_point",10,frontCallback);
	ros::Subscriber left_sub = nh.subscribe("left_point",10,leftCallback);
	ros::Subscriber imu_sub = nh.subscribe("imu_data",10,imuCallback);
	ros::Publisher coordinate = nh.advertise<geometry_msgs::Point>("coordinate",10);
	ros::Publisher pointstamped_pub = nh.advertise<geometry_msgs::PointStamped>("my_pointstamped_pub",100);
	ros::Rate loop_rate(20);
	geometry_msgs::Point coordinate_msg;
	geometry_msgs::PointStamped pointstamped_msg;

	while(ros::ok())
	{
        //cout<<R_b2w<<endl;
		coordinate_msg.x = Px;
		coordinate_msg.y = Py;
		coordinate_msg.z = Height;
		coordinate.publish(coordinate_msg);
		pointstamped_msg.header.frame_id = "my_frame";
		pointstamped_msg.header.stamp = ros::Time::now();
		pointstamped_msg.point = coordinate_msg;
		pointstamped_pub.publish(pointstamped_msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
}

