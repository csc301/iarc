 // if(height>HEIGHT+HEIGHT_THRESHOLD)hover_cmd.linear.z = DOWN_PWM;
           // if(height<HEIGHT-HEIGHT_THRESHOLD)hover_cmd.linear.z = UP_PWM;
            //hover_publisher.publish(hover_cmd);
 
         /*   sum_dt+=dt;
            if(sum_dt<5)
            { 
              sum_x_ref=0;
              sum_y_ref=0;
            }
            if((sum_dt>5)&&(sum_dt<10))
            {
              sum_x_ref=SQARE;
              sum_y_ref=0;
            }
            if((sum_dt>10)&&(sum_dt<15))
            {
              sum_x_ref=SQARE;
              sum_y_ref=SQARE;
            }
            if((sum_dt>15)&&(sum_dt<20))
            {
              sum_x_ref=0;
              sum_y_ref=SQARE;
            }
            if((sum_dt>20)&&(sum_dt<25))
            {
              sum_x_ref=0;
              sum_y_ref=0;
              sum_dt=0;
            }

              sum_x_ref=0;
              sum_y_ref=0;
        */
    


       /*pose_msg.pose.position.x = sum_x;
       pose_msg.pose.position.y = sum_y;
       pose_msg.pose.position.z = height;

       pose_msg.pose.orientation.x = 1;
       pose_msg.pose.orientation.y = 0;
       pose_msg.pose.orientation.z = 0;
       pose_msg.pose.orientation.w = 0;
              //point_publisher.publish(point_msg);
       //path_msg.poses[0] = pose_msg;
       //i++;
       //path_publisher.publish(path_msg);
       */


/*bool hover_srv_callback(px4flow_hover::hover_srv::Request &req , px4flow_hover::hover_srv::Response &resp)
{
    resp.hover_srv = hover_cmd;
    return true;
}*/


    //hover_srv = n.advertiseService("hover_srv", hover_srv_callback);

#define HEIGHT 1.5
#define HEIGHT_THRESHOLD 0.10
#define DOWN_PWM -60
#define UP_PWM 30
