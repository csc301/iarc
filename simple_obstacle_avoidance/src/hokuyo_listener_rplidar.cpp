/**************************/
//liuhyuu
//2014_
/*************************/
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "math.h"

ros::Publisher hokuyoPub01,hokuyoPub02;
std_msgs::Bool flag;
geometry_msgs::Twist action;
double min = 1;
int count = -1;
int original_x = 0.2;
double step = 1;
double pi = 3.1415926;

double computeAngle(int min_range_angle,int num_readings)
{
	double angle;
	double theta;
	ROS_INFO("%d %d",min_range_angle,num_readings);	
	angle = (double)min_range_angle/(double)num_readings * 360 - 90;//大头朝向为0
	ROS_INFO("%lf",angle);
	theta = angle * pi /180;
	return theta;
}
void scanCallback(const sensor_msgs::LaserScan laser)
{
    	unsigned int num_readings = laser.ranges.size();
    	num_readings = num_readings - 1;
	double ranges1[6][num_readings],ranges[num_readings];
	//bool flag = true;
	//double pi = 3.1415926;
	//ROS_INFO("HERE");
	//范围值存储
	for(int i = 0;i <= num_readings; i++){					//中值滤波
		
		ranges1[count][i] = laser.ranges[i];
	}
	
	for(int i = 0;i <= num_readings; i++){
		ranges1[5][i] = 99;
	}
	if(count == 4){
		for(int i = 0;i <= num_readings; i++){
			double temp;
			for(int j = 0;j < 5; j++){
				for(int k = 0; k < 5 - j; k++){
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
		double theta; //距离飞行器最近的点的角度
		double theta2; //action所使用的theta值
	
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
			theta = computeAngle(min_range_angle,num_readings);
			theta2 = theta + 180;			
			action.linear.x = step * sin(theta2);
			action.	linear.y = - (step * cos(theta2));
			hokuyoPub02.publish(action);
			//ROS_INFO("%lf",theta);		
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
 	ros::init(argc, argv, "Obstacle_Avoidance");
  	ros::NodeHandle n;
 	hokuyoPub01 = n.advertise<std_msgs::Bool>("ObstacleAvoid",1);
 	flag.data = false;
 	hokuyoPub02 = n.advertise<geometry_msgs::Twist>("cmd_vel",10);
	ROS_INFO("Obstacle_Avoidance");

	ros::Subscriber hokuyoSubscriber = c.subscribe("/scan", 10, scanCallback);
			
	action.angular.x = 0;
	action.angular.y = 0;
	action.angular.z = 0;
	action.linear.x = 0;
	action.linear.y = 0;
	action.linear.z = 0;
		
 	ros::spin();
  	return 0;
}
