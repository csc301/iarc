/**************************/
//liuhyuu
//2015.4
//modified by ycc
//2015.5
/*************************/
#include "ros/ros.h"
#include "iostream"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "simple_obstacle_avoidance/avo_srv.h"
#include "math.h"

#define PI  3.14159
#define RPLIDAR 360       //rplidar 360
#define BUBBLE_TIMES 5
#define AVO_THRE 1.2
#define STEP 70

using namespace std;

std_msgs::Bool avo_flag ;
ros::Publisher obs_avo_pub,avo_flag_pub;
geometry_msgs::Twist action;
double min_range=AVO_THRE;
unsigned int min_range_angle=0;
int counter = 0;
unsigned int num_readings = RPLIDAR;
double all_ranges[BUBBLE_TIMES][RPLIDAR],filter_ranges[RPLIDAR];

void laser_scan_callback(const sensor_msgs::LaserScan laser)
{
	for(int i = 0;i < num_readings; i++)
      {			
	     all_ranges[counter][i] = laser.ranges[i];
	}

	if(counter == BUBBLE_TIMES-1)
        {
             counter = 0;
             for(int i = 0;i < num_readings; i++)
             {
		bool bubble_flag=false;
		for(int j = 1;j <= (BUBBLE_TIMES-1); j++)
            {
		   for(int k = 1; k <= (BUBBLE_TIMES-j); k++)
                {
		       if(all_ranges[BUBBLE_TIMES-j][i] < all_ranges[BUBBLE_TIMES-j-1][i])
                    {
		             double temp = all_ranges[BUBBLE_TIMES-j-1][i];
			      all_ranges[BUBBLE_TIMES-j-1][i] = all_ranges[BUBBLE_TIMES-j][i];
			      all_ranges[BUBBLE_TIMES-j][i] = temp;
                          bubble_flag = true;
		       }
		   }
                   if(bubble_flag==false)break;
                   bubble_flag = false;		
		}
	       filter_ranges[i] = all_ranges[(BUBBLE_TIMES-1)/2][i];
             }  
		
             for(int i=0;i<num_readings;i++)
             {
	       if(filter_ranges[i] < min_range)
	       {
	          min_range = filter_ranges[i];
		  min_range_angle = i;			
	       }
             }  
            
             cout <<min_range<<" "<<min_range_angle<<endl;

             if(filter_ranges[min_range_angle]<AVO_THRE)
             {
                avo_flag.data=true;
                avo_flag_pub.publish(avo_flag);
                action.linear.x = int(-1*STEP*cos((min_range_angle/180.0)*PI));
                action.linear.y = int(STEP*sin((min_range_angle/180.0)*PI));
                action.angular.x=1;
                obs_avo_pub.publish(action);
                //cout <<action.linear.x<<" "<<action.linear.y<<endl;
              }
             else
             {
                avo_flag.data=false;
                avo_flag_pub.publish(avo_flag);
                action.linear.x = 0;
                action.linear.y = 0;
                action.linear.z = 0; 
                action.angular.x=0;
                action.angular.y=0;
                action.angular.z=0;
                obs_avo_pub.publish(action);
             }
	}
	else
	{
	     counter ++;
	}
        min_range=AVO_THRE;
}

/*bool avo_srv_callback(simple_obstacle_avoidance::avo_srv::Request &req , simple_obstacle_avoidance::avo_srv::Response &resp)
{
    resp.avo_action = action;
    return true;
}*/

int main(int argc, char **argv)
{	 	
 	ros::init(argc, argv, "Obstacle_Avoidance_node");
  	ros::NodeHandle n;
 	obs_avo_pub = n.advertise<geometry_msgs::Twist>("/avo_cmd",10);
        avo_flag_pub = n.advertise<std_msgs::Bool>("/obstacle_avoid_flag",10);
	
	ros::Subscriber hokuyo_sub = n.subscribe("/scan", 10, laser_scan_callback);

       //ros::ServiceServer avo_srv = n.advertiseService("/avo_srv", avo_srv_callback);

 	ros::spin();
  	return 0;
}
