#include "global.h"
using namespace std;
using namespace cv;
CGlobal::CGlobal(ros::NodeHandle nh):
	nh_(nh)
{
    ROS_INFO("Starting CGlobal...");
    imu_sub = nh_.subscribe("imu_sub",10,&CGlobal::imuCallback,this);
    front_sub = nh_.subscribe("front_point",10,&CGlobal::frontCallback,this);
    behind_sub = nh_.subscribe("behind_point",10,&CGlobal::behindCallback,this);
    left_sub = nh_.subscribe("left_point",10,&CGlobal::leftCallback,this);
    right_sub = nh_.subscribe("right_point",10,&CGlobal::rightCallback,this);
	Rz = Mat(3,3,CV_64FC1);
	Ry = Mat(3,3,CV_64FC1);
	Rx = Mat(3,3,CV_64FC1);
	R_w2b = Mat(3,3,CV_64FC1);
	R_b2w = Mat(3,3,CV_64FC1);
	//R_w2b = Mat::eye(3,3,CV_64FC1);
    //R_b2w = Mat::eye(3,3,CV_64FC1);
	/*
	Vec3d Ia,Ib,Ic,Id,Ie;
	Ia[0] = 1.0;Ia[1] = 1.0;Ia[2] = -1.0;  //Ia is a vector in body(pi) frame, (u,v) * R_image2camera * R_camera2bodyframe = Ia;
	Ib[0] = -1.0;Ib[1] = 1.0;Ib[2] = -1.0;
	Ic[0] = 1.0;Ic[1] = -1.0;Ic[2] = -1.0;
	Id[0] = -1.0;Id[1] = -1.0;Id[2] = -1.0;
	Ie[0] = -1.0;Ie[1] = 0.0;Ie[2] = -1.0;
	left[0] = Ia;
	left[1] = Ib;
	right[0] = Ic;
	right[1] = Id;
	behind[0] = Ie;
	*/
	Vec3d rtn;
	ros::Rate loopRate(20);
	while(ros::ok())
    {
        /*
        bool noFront_,noBehind_,noLeft_,noRight_;
        Vec3d front_[2],behind_[2],left_[2],right_[2];
        noFront_ = noFront;noBehind_ = noBehind;noLeft_ = noLeft;noRight_ = noRight;
        front_[0] = front[0];front_[1] = front[1];behind_[0] = behind[0];behind_[1] = behind[1];
        */
        if (!noBehind && !noLeft && !noRight)
        {
            ROS_INFO("noFront....");
            rtn = calcGlobalPosition(left[0],left[1],right[0],right[1],behind[0]);
            Px = rtn[0];
            Py = rtn[1];
            Pz = rtn[2];
        }
        else if(!noFront && !noLeft && !noRight)
        {
            ROS_INFO("noBehind....");
            rtn = calcGlobalPosition(right[0],right[1],left[0],left[1],front[0]);
            Px = SQUARE - rtn[0];
            Py = SQUARE - rtn[1];
            Pz = rtn[2];
        }
        else if (!noFront && !noBehind && !noRight)
        {
            ROS_INFO("noLeft....");
            rtn = calcGlobalPosition(behind[0],behind[1],front[0],front[1],right[0]);
            Px = SQUARE - rtn[1];
            Py = rtn[0];
            Pz = rtn[2];
        }
        else if (!noFront && !noBehind && !noLeft)
        {
            ROS_INFO("noRight....");
            rtn = calcGlobalPosition(front[0],front[1],behind[0],behind[1],left[0]);
            Px = rtn[1];
            Py = SQUARE - rtn[0];
            Pz = rtn[2];
        }
        ROS_INFO("Px= %lf, Py= %lf, Pz= %lf",Px,Py,Pz);
        loopRate.sleep();
        ros::spinOnce();
    }
}

CGlobal::~CGlobal(){cout<<"destroying CGlobal..."<<endl;}

void CGlobal::initializeRotationMatrix()
{
	double rz[]={cos(yaw),sin(yaw),0,
		-sin(yaw),cos(yaw),0,
		0,0,1};
	double ry[]={cos(roll),0,-sin(roll),
		0,1,0,
		sin(roll),0,cos(roll)};
	double rx[]={1,0,0,
		0,cos(pitch),sin(pitch),
		0,-sin(pitch),cos(pitch)};
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
}

Vec3d CGlobal::calcGlobalPosition(Vec3d Ia,Vec3d Ib,Vec3d Ic,Vec3d Id,Vec3d Ie)
{
	Mat n1b,n2b;
	Mat n1w,n2w;
	Vec3d rtn;
	double px,py,pz;
	n1b = Mat(3,1,CV_64FC1);
	n2b = Mat(3,1,CV_64FC1);
	n1w = Mat(3,1,CV_64FC1);
	n2w = Mat(3,1,CV_64FC1);
	n1b = Mat(Ia).cross(Mat(Ib));
	if (n1b.at<double>(2,0) < 0) n1b = Mat(Ib).cross(Mat(Ia));
	n2b = Mat(Id).cross(Mat(Ic));
	if (n2b.at<double>(2,0) < 0) n2b = Mat(Ic).cross(Mat(Id));
	//cout<<"n1b= "<<n1b<<endl<<"n2b= "<<n2b<<endl;
	n1w = R_b2w * n1b;
	n2w = R_b2w * n2b;
	//cout<<"n1w= "<<n1w<<endl<<"n2w= "<<n2w<<endl;
	double temp1 = abs(n1w.at<double>(1,0) / n1w.at<double>(2,0));
	double temp2 = abs(n2w.at<double>(1,0) / n2w.at<double>(2,0));
	//cout<<"temp1= "<<temp1<<endl<<"temp2= "<<temp2<<endl;
	py = SQUARE*temp1/(temp1+temp2);
	pz = temp2*py;
	//cout<<"py= "<<py<<endl<<"pz= "<<pz<<endl;
	Mat Iew = R_b2w*Mat(Ie);
	px = abs(Iew.at<double>(0,0)/Iew.at<double>(2,0))*pz;
	//cout<<"px= "<<px<<endl;
	rtn[0] = px;
	rtn[1] = py;
	rtn[2] = pz;
	return rtn;
}


	void CGlobal::imuCallback(const sensor_msgs::ImuConstPtr& msg)
	{
	    pitch = msg->orientation.x;
	    roll = msg->orientation.y;
	    yaw = msg->orientation.z;
	    initializeRotationMatrix();
	};
    void CGlobal::frontCallback(const line_detect::pointsConstPtr& msg)
    {
        noFront = msg->havenopoints;
        front[0][0] = msg->point1.x;
        front[0][1] = msg->point1.y;
        front[0][2] = msg->point1.z;
        front[1][0] = msg->point2.x;
        front[1][1] = msg->point2.y;
        front[1][2] = msg->point2.z;
    };
	void CGlobal::behindCallback(const line_detect::pointsConstPtr& msg)
	{
        noBehind  = msg->havenopoints;
        behind[0][0] = msg->point1.x;
        behind[0][1] = msg->point1.y;
        behind[0][2] = msg->point1.z;
        behind[1][0] = msg->point2.x;
        behind[1][1] = msg->point2.y;
        behind[1][2] = msg->point2.z;
	};
	void CGlobal::leftCallback(const line_detect::pointsConstPtr& msg)
	{
        noLeft = msg->havenopoints;
        left[0][0] = msg->point1.x;
        left[0][1] = msg->point1.y;
        left[0][2] = msg->point1.z;
        left[1][0] = msg->point2.x;
        left[1][1] = msg->point2.y;
        left[1][2] = msg->point2.z;
	};
	void CGlobal::rightCallback(const line_detect::pointsConstPtr& msg)
	{
        noRight = msg->havenopoints;
        right[0][0] = msg->point1.x;
        right[0][1] = msg->point1.y;
        right[0][2] = msg->point1.z;
        right[1][0] = msg->point2.x;
        right[1][1] = msg->point2.y;
        right[1][2] = msg->point2.z;
	};
