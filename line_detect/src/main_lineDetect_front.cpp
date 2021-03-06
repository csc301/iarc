#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "linefinder.h"
#define PI 3.1415926
using namespace std;
using namespace cv;
/*
Mat image;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //cv::imshow("view", cv_bridge::toCvShare(msg, "mono8")->image);
    cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(image);
    //imshow(image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
*/

int main(int argc,char ** argv)
{
	ros::init(argc,argv,"line_detect_node_front");
	ros::NodeHandle nh;
	ros::Publisher pub_front_point = nh.advertise<line_detect::points>("front_point",10);

	image_transport::ImageTransport it(nh);
	image_transport::Publisher img_pub = it.advertise("front_image",1);
	sensor_msgs::ImagePtr msg;


	//image_transport::ImageTransport it(nh);
	//image_transport::Subscriber sub = it.subscribe("front_image", 100, imageCallback);


	line_detect::points front_point;
	int capture_number;
	double thre_angel_;
	double thre_DistanceOfLines_;
	double fx,fy,cx,cy;
	double f;
	if(!nh.getParam("capture_number",capture_number)) capture_number = 0;
	if(!nh.getParam("thre_angel_",thre_angel_)) thre_angel_ = 30.0;
	if(!nh.getParam("thre_DistanceOfLines_",thre_DistanceOfLines_)) thre_DistanceOfLines_ = 30;
	if(!nh.getParam("fx",fx))fx = 709.458979;
	if(!nh.getParam("fy",fy))fy = 706.692079;
	if(!nh.getParam("cx",cx))cx = 346.864677;
	if(!nh.getParam("cy",cy))cy = 231.634484;
	f = (fx+fy)/2.0;
	VideoCapture cap(capture_number);
	Mat roi;//roi_gray;
	Mat image;
	//Mat contours,contoursInv;
	// Create LineFinder instance
	LineFinder ld;
	int line_length = 400;
	int line_gap = 20;
	int minvote = 100;
	//int canny_L_thre = 125;
	//int canny_H_thre = 350;
	string winname = "Params_front";
	namedWindow(winname,WINDOW_NORMAL);
	ld.setThre_DistanceOfLines(thre_DistanceOfLines_);
	ld.setThre_angel(thre_angel_);
	createTrackbar("min_length",winname,&line_length,700);
	createTrackbar("max_gap",winname,&line_gap,100);
	createTrackbar("minvote",winname,&minvote,300);
	//createTrackbar("canny_H",winname,&canny_H_thre,400);
	//createTrackbar("canny_L",winname,&canny_L_thre,200);
	if(!cap.isOpened()){cout<<"cap open failed..."<<endl;return 0;}
	cap.read(image);
	int thresh_red = 100;
	int thresh_green = 50;
	vector<Mat> mv;
	while (ros::ok())
	{
		ld.setLineLengthAndGap(line_length,line_gap); //length = 100,gap = 20
		ld.setMinVote(minvote); //minvote = 100
		cap.read(image);

		//roi = Mat(image,Rect(0,(int)(image.rows/2),image.cols,(int)(image.rows/2)));
		Mat roi_test;
		if(!image.empty())
		{
			image.copyTo(roi_test);
			cv::Mat_<cv::Vec3b>::iterator it = roi_test.begin<cv::Vec3b>();   //初始位置迭代器
			cv::Mat_<cv::Vec3b>::iterator itend = roi_test.end<cv::Vec3b>();  //终止位置迭代器
			for(;it!=itend;++it)
			{
			 if( ((*it)[1]>(0.5*((*it)[0]+(*it)[2])+thresh_green)) && ((*it)[1]>((*it)[0]+thresh_green-10)) && ((*it)[1]>((*it)[2]+thresh_green-10)) )
			   {                                                                                            //BGR GGG  fornight 90 60 60  120 100 100
			     (*it)[0]=0;
			 (*it)[1]=0;
			 (*it)[2]=0;
			   }
			  else if( (*it)[2]>(0.5*((*it)[0]+(*it)[1])+thresh_red) && ((*it)[2]>((*it)[0]+thresh_red-10)) && ((*it)[2]>((*it)[1]+thresh_red-10)) )
			   {                                                                                            //BGR  RRR forsun 70 60 60 90 80 80
			     (*it)[0]=255;                                                                              //sunday everychannel value is big
			 (*it)[1]=0;
			 (*it)[2]=0;
			   }
			  else
			{
			 (*it)[0]=0;
			 (*it)[1]=0;
			 (*it)[2]=0;
			}
			}
			split(roi_test,mv);
			Mat binary;
			mv[0].copyTo(binary);
			//imshow("red_binary",binary);
			//cvtColor(roi,roi_gray,CV_RGB2GRAY);
			//Canny(roi_gray,contours,canny_L_thre,canny_H_thre);   //canny_L_thre = 125,canny_H_thre = 350
			//threshold(contours,contoursInv,128,255,cv::THRESH_BINARY_INV);
			//imshow("Canny Contours",contours);
			ld.findLines(binary);
			ld.selectGoodLines();
			ld.drawDetectedLines(image);
			//imshow("Detected Lines with HoughP_front",image);

			msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
			img_pub.publish(msg);


			if(ld.lines.size()>0)
			{
			//把图片坐标系下的坐标，先转到摄像头坐标系，再转到机体轴坐标系。这里发布的坐标已经 是机体轴坐标系下的坐标值了！！！！
			//cout<<ld.lines[0][0]<< " " <<ld.lines[0][1]<<endl;
				cout<<-ld.lines[0][0]+cx<<" "<<ld.lines[0][1]-cy<<endl;
				front_point.point1.y = -ld.lines[0][0]+cx;
				front_point.point1.z = -(ld.lines[0][1]-cy);
				front_point.point1.x = f;
				front_point.point2.y = -ld.lines[0][2]+cx;
				front_point.point2.z = -(ld.lines[0][3]-cy);
				front_point.point2.x = f;
				front_point.havenopoints = false;
			}
			else
			{
				front_point.point1.x = 0;
				front_point.point1.z =0;
				front_point.point1.y = 0;
				front_point.point2.x = 0;
				front_point.point2.z = 0;
				front_point.point2.y = 0;
				front_point.havenopoints = true;
			}
			pub_front_point.publish(front_point);
		}
		waitKey(20);
		ros::spinOnce();
	}





/*
std::vector<cv::Vec4i>::const_iterator it2= li.begin();
	while (it2!=li.end()) {
		std::cout << "(" << (*it2)[0] << ","<< (*it2)[1]<< ")-("
			     << (*it2)[2]<< "," << (*it2)[3] << ")" <<std::endl;
		++it2;
	}
*/
}
/*
// Hough tranform for line detection
std::vector<cv::Vec2f> lines;
cv::HoughLines(contours,lines,1,PI/180,100);
// Draw the lines
cv::Mat result(contours.rows,contours.cols,CV_8U,cv::Scalar(255));
image.copyTo(result);

std::cout << "Lines detected: " << lines.size() << std::endl;

std::vector<cv::Vec2f>::const_iterator it= lines.begin();
while (it!=lines.end()) {

float rho= (*it)[0];   // first element is distance rho
float theta= (*it)[1]; // second element is angle theta

if (theta < PI/4. || theta > 3.*PI/4.) { // ~vertical line

// point of intersection of the line with first row
cv::Point pt1(rho/cos(theta),0);
// point of intersection of the line with last row
cv::Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
// draw a white line
cv::line( result, pt1, pt2, cv::Scalar(255), 1);

} else { // ~horizontal line

// point of intersection of the line with first column
cv::Point pt1(0,rho/sin(theta));
// point of intersection of the line with last column
cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
// draw a white line
cv::line( result, pt1, pt2, cv::Scalar(255), 1);
}

std::cout << "line: (" << rho << "," << theta << ")\n";

++it;
}

// Display the detected line image
cv::namedWindow("Detected Lines with Hough");
cv::imshow("Detected Lines with Hough",result);
*/
