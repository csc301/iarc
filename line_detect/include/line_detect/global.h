#ifndef GLOBAL_H
#define GLOBAL_H
#include <opencv2/opencv.hpp>
#include <math.h>
#include "ros/ros.h"
#include <iostream>
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Point.h"
#include "line_detect/points.h"
using namespace std;
using namespace cv;
#define PI 3.1415926
#define SQUARE 20.0
class CGlobal
{
public:
	double Px,Py,Pz;
	CGlobal(ros::NodeHandle nh);
	virtual ~CGlobal();
private:
    ros::NodeHandle nh_;
    ros::Subscriber front_sub,behind_sub,left_sub,right_sub;
    ros::Subscriber imu_sub;
	double yaw;
	double roll;
	double pitch;
	Mat Rz;
	Mat Ry;
	Mat Rx;
	Mat R_w2b,R_b2w;
	Vec3d left[2],right[2],front[2],behind[2];
	bool noLeft,noRight,noFront,noBehind;
	void initializeRotationMatrix();
	Vec3d calcGlobalPosition(Vec3d Ia,Vec3d Ib,Vec3d Ic,Vec3d Id,Vec3d Ie);
	void imuCallback(const sensor_msgs::ImuConstPtr& msg);
	void frontCallback(const line_detect::pointsConstPtr& msg);
	void behindCallback(const line_detect::pointsConstPtr& msg);
	void leftCallback(const line_detect::pointsConstPtr& msg);
	void rightCallback(const line_detect::pointsConstPtr& msg);

};
#endif // GLOBAL_H
