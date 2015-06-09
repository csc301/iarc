{
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
				hokuyoPub02.publish(action);
				//point.x = 2;
			}
		}

