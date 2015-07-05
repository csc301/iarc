#ifndef multi_object_H
#define multi_object_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt16MultiArray.h>
#include <geometry_msgs/Polygon.h>
#include "multi_object/RoiArray.h" 
#include <opencv2/opencv.hpp>

#include <std_msgs/Header.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <vector>
#include <algorithm>

using namespace std;
using namespace cv;

class multi_object_
{
   public:
      multi_object_(ros::NodeHandle nh);
      virtual ~multi_object_();

   private:
      ros::NodeHandle nh_;
      ros::Subscriber image_subsciber;
      ros::Publisher  rois_publisher,roi_publisher,edge_publisher;

      cv_bridge::CvImagePtr cv_ptr;
      sensor_msgs::RegionOfInterest rois_msg,roi_msg,edge_msg;
      multi_object::RoiArray roi_arr;

      Mat frame_raw;
      Mat frame_new;
      vector<Mat> mv;
      vector< vector<Point> > contours;

      void getSizeContours( vector<vector<Point> >& con);
      void image_subcallback(const sensor_msgs::Image::ConstPtr& msg);

};

multi_object_::multi_object_(ros::NodeHandle nh):
nh_(nh)
{
  ROS_INFO("Starting multi_object!");

  std::string camera_topic;
    if (!nh_.getParam ("from_camera/camera_topic_name", camera_topic))  
      camera_topic = "/usb_camera/image_rect_color";
  image_subsciber = nh_.subscribe(camera_topic.c_str(), 10, &multi_object_::image_subcallback,this);
  rois_publisher = nh_.advertise<multi_object::RoiArray>("rois_list",10);
  roi_publisher = nh_.advertise<sensor_msgs::RegionOfInterest>("sigle_roi",10);
  edge_publisher = nh_.advertise<sensor_msgs::RegionOfInterest>("red_edge",10);

  ros::Rate loop_rate(40);
  namedWindow("image_rect_color",WINDOW_NORMAL);    //WINDOW_NORMAL  CV_WINDOW_AUTOSIZE
  moveWindow("image_rect_color",0,0);
  namedWindow("image_rect_color_binary",CV_WINDOW_AUTOSIZE);
  moveWindow("image_rect_color_binary",700,0);
  int max_thresh = 255;
  int thresh_red = 90;
  int thresh_green = 250;
  while(ros::ok())
  {
    ros::spinOnce();
    if(!frame_raw.data)
    {
      ros::spinOnce();
      loop_rate.sleep();
      ROS_INFO("Frame is empty!!!!");
      continue;
    }
    else
    {
      imshow("image_rect_color",frame_raw);
      waitKey(3);
      cv::Mat_<cv::Vec3b>::iterator it = frame_new.begin<cv::Vec3b>();   //初始位置迭代器
      cv::Mat_<cv::Vec3b>::iterator itend = frame_new.end<cv::Vec3b>();  //终止位置迭代器

      struct timeval start;
      struct timeval end;
      clock_t begin,finish;
      double duration,time_use;

      begin=clock();
      for(;it!=itend;++it)
      {
         if( ((*it)[1]>(0.5*((*it)[0]+(*it)[2])+thresh_green)) && ((*it)[1]>((*it)[0]+thresh_green-10)) && ((*it)[1]>((*it)[2]+thresh_green-10)) )   
           {                                                                                            //BGR GGG  fornight 90 60 60  120 100 100
             (*it)[0]=255;
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
      finish=clock();
      duration = (double)(finish - begin) / CLOCKS_PER_SEC;
      split(frame_new,mv);
      Mat binary;
      mv[0].copyTo(binary);

      begin=clock();
      gettimeofday(&start,NULL); //gettimeofday(&start,&tz);结果一样
      findContours(mv[0], contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  //会改变mv[0],连通区域判断
      gettimeofday(&end,NULL); 
      finish=clock();
      //duration = (double)(finish - begin) / CLOCKS_PER_SEC;
      time_use=(end.tv_sec-start.tv_sec)+(end.tv_usec-start.tv_usec)/1000000.0;
      //cout<<time_use<<"  "<<duration<<endl;

      getSizeContours(contours);                                              //大小过滤
      //cout << contours.size() <<endl;
      //Mat result(mv[0].size(), CV_8U, Scalar(255));  
      //drawContours(mv[0], contours, i, Scalar(255), 2);   
      for(int i=0;i<contours.size();i++)
        {
          Rect r = boundingRect(Mat(contours[i]));                //长方形包络

         if(r.height>2*r.width || r.width>2*r.height)
         {
             edge_msg.x_offset=r.x;
             edge_msg.y_offset=r.y;
             edge_msg.height=r.height;
             edge_msg.width=r.width;
             edge_publisher.publish(edge_msg);
         }
         else
         {
            roi_msg.x_offset=r.x;
            roi_msg.y_offset=r.y;
            roi_msg.height=r.height;
            roi_msg.width=r.width;
            roi_publisher.publish(roi_msg);
         }
	  rois_msg.x_offset=r.x;
	  rois_msg.y_offset=r.y;
	  rois_msg.height=r.height;
	  rois_msg.width=r.width;
          //////////cout << r.x <<" "<< r.y <<" "<< r.width <<" "<< r.height <<endl;

          roi_arr.list.push_back(rois_msg);
          drawContours(binary, contours, i, Scalar(155), 2);      // -1 表示所有轮廓    //255白色  
          rectangle(binary,r,Scalar(255),2);   
        }
   
      imshow("image_rect_color_binary",binary);  //BGR  

      createTrackbar( " Threshold_green:", "image_rect_color_binary", &thresh_green, max_thresh, NULL );
      createTrackbar( " Threshold_red:", "image_rect_color_binary", &thresh_red, max_thresh, NULL );

      rois_publisher.publish(roi_arr);
      roi_arr.list.clear();

      loop_rate.sleep();
    }
  }
}


multi_object_::~multi_object_()
{  
  ROS_INFO("Destroying multi_object!");
  destroyAllWindows();     
}

void multi_object_::image_subcallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  cv_ptr->image.copyTo(frame_raw);
  frame_raw.copyTo(frame_new); 
}

void multi_object_::getSizeContours(vector< vector<Point> >& con)
{  
    int cmin = 30;   // 最小轮廓长度  
    int cmax = 1800;   // 最大轮廓长度  
    vector< vector<Point> >::iterator itc = con.begin();   //嵌套模板要把尖括号分开
    while(itc != con.end())  
    {  
        if((itc->size()) < cmin || (itc->size()) > cmax)  
        {  
           itc = con.erase(itc);  
        }  
        else ++ itc;  
    }  
}  

#endif
