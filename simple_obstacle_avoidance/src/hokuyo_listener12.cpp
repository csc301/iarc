/**************************/
//liuhyuu
//2015_4
/*************************/
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "math.h"

ros::Publisher hokuyoPub01,hokuyoPub02;
std_msgs::Bool flag;
geometry_msgs::Twist action;
double min = 2;
int count = -1;
int original_x = 0.2;
sensor_msgs::LaserScan velocity;

void velocitycallback(const sensor_msgs::LaserScan laser1)
{
	velocity = laser1;
}
void scanCallback(const sensor_msgs::LaserScan laser)
{
    	unsigned int num_readings = laser.ranges.size();
    	num_readings = num_readings - 1;
	double ranges1[10][num_readings],ranges[num_readings];
	//bool flag = true;
	double pi = 3.1415926;
	//ROS_INFO("HERE");
	//范围值存储
	for(int i = 0;i <= num_readings; i++){					//中值滤波
		
		ranges1[count][i] = laser.ranges[i];
	}
	
	for(int i = 0;i <= num_readings; i++){
		ranges1[9][i] = 99;
	}
	if(count == 8){
		for(int i = 0;i <= num_readings; i++){
			double temp;
			for(int j = 0;j < 9; j++){
				for(int k = 0; k < 9 - j; k++){
					if( ranges1[k][i]>ranges1[k+1][i]){
						temp = ranges1[k][i];
						ranges1[k][i] = ranges1[k+1][i];
						ranges1[k+1][i] = temp;
					}
				}		
			}
			ranges[i] = ranges1[4][i];
		}
		//决定避障方向
		double min_range = min;
		int min_range_angle;
	
		for(int i=0;i<=num_readings;i++){
			if(ranges[i] < min_range)
			{
				min_range = ranges[i];
				min_range_angle = i;			
			}
		
		}
		printf("here min_range = %lf,min_range_angle = %u\n",min_range,min_range_angle);
		
		if(min_range<min)
		{
			//hokuyoPublish.publish(1);
			flag.data = true;
			if((0.6*num_readings)<min_range_angle||min_range_angle<(0.4*num_readings))
			{
				if(min_range_angle>=(num_readings/2))
				{				
				action.angular.z = 1;
				}
				else if(min_range_angle<(num_readings/2))
				{
				action.angular.z = -1;
				}
				hokuyoPub02.publish(action);
			}
			else if(min_range_angle<(num_readings/2))
			{
			
				ROS_INFO("distacne:%lf,angle:%u,left", min_range,min_range_angle);  
				//left
				//double angle = min_range_angle;
				action.angular.z = -0.08;
				action.linear.x = -velocity.ranges[min_range_angle]*(1/min_range)*0.1;
				//ROS_INFO("ex1 = %lf",ex1);
				//ROS_INFO("cos = %lf",cos((double)min_range_angle/((double)num_readings/2)*(pi/2)));
				if(action.linear.x > 0.6)
				{
					action.linear.x = 0.6;
				}
				else{}
				hokuyoPub02.publish(action);
			
			}
			else if(min_range_angle>=(num_readings/2)&&min_range_angle<=num_readings - 1)
			{
				//ROS_INFO("distacne：%lf,right", (float)min_range);
				ROS_INFO("distacne:%lf,angle:%u,right", (float)min_range,min_range_angle);  
				//right
				action.angular.z = 0.08;
				action.linear.x = -velocity.ranges[min_range_angle]*(1/min_range)*0.1;
				ROS_INFO("action.x = %lf",action.linear.x);	
				if(action.linear.x > 0.6)
				{
					action.linear.x = 0.6;
				}
				else{}			
				hokuyoPub02.publish(action);
				//point.x = 2;
			}
			else
			{
				//ROS_INFO("HERE");
			}

				
		
		
		}
		else
		{
			flag.data = false;
			ROS_INFO("NO ACTION");
			
		}
	
		//sleep(1);
		hokuyoPub01.publish(flag);
		action.angular.x = 0;
	action.angular.y = 0;
	action.angular.z = 0;
	action.linear.x = 0;
	action.linear.y = 0;
	action.linear.z = 0;
  		flag.data = false;
		
		count = 0;
	}
	else
	{
	 	count++;
	 	//ROS_INFO("COUNT %d",count);
	}
}

int main(int argc, char **argv)
{
	 	
 	ros::init(argc, argv, "hokuyo_listener");
  	ros::NodeHandle a;
 	ros::NodeHandle b;
  	ros::NodeHandle c;
	ros::NodeHandle d;
 	hokuyoPub01 = a.advertise<std_msgs::Bool>("ObstacleAvoid",1);
 	flag.data = false;
 	hokuyoPub02 = b.advertise<geometry_msgs::Twist>("cmd_vel",10);
	ROS_INFO("Obstacle_Avoidance");
	
  	  	//ROS_INFO("HERE");
	ros::Subscriber velocitySubscriber = d.subscribe("/Velocity",10,velocitycallback);  	
	ros::Subscriber hokuyoSubscriber = c.subscribe("/scan", 10, scanCallback);
		
	action.angular.x = 0;
	action.angular.y = 0;
	action.angular.z = 0;
	action.linear.x = 0;
	action.linear.y = 0;
	action.linear.z = 0;
		
 	ros::spin();
  	//ROS_INFO("HERE OVER");
 	
  	return 0;
}
/*void obstacleAvoidance(const sensor_msgs::LaserScan laser)
{
	
}
*/



