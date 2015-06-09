#ifndef CMD_VEL_SERIAL_H
#define CMD_VEL_SERIAL_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "time.h"
#include <termios.h> 
#include <errno.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <cstdio>
#include <cstring>

using namespace std;

class cmd_vel_serial
{
public:
	cmd_vel_serial(ros::NodeHandle nh);
	~cmd_vel_serial();

	ros::NodeHandle cmd_vel_nh;
	ros::Subscriber cmd_vel_sub;

        int serialfd,i,serialdata[10];
        string cmd_vel_serial_port;
        struct termios oldtio;

	int set_term(int fd, int nSpeed, int nBits, char nEvent, int nStop);
        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
};

cmd_vel_serial::cmd_vel_serial(ros::NodeHandle nh):cmd_vel_nh(nh)
{
	if (!cmd_vel_nh.getParam ("to_stm32/stm32_serial_port", cmd_vel_serial_port)) 
        cmd_vel_serial_port = "/dev/ttyUSB2";
    
    serialfd = open(cmd_vel_serial_port.c_str (), O_RDWR);
    if(-1 == serialfd)
      {
        perror("open serial error!");
        exit(1);
      }
    if((tcgetattr(serialfd, &oldtio)) != 0)
      {
        perror("SetupSerial");
        //return -1;
      }
    if((i = set_term(serialfd, 115200, 8, 'N', 1)) < 0)
      {
        perror("set_term error");
        exit(1);
      }

    cmd_vel_sub = cmd_vel_nh.subscribe("cmd_vel", 100, &cmd_vel_serial::cmd_vel_callback,this);
    ros::spin();
}

cmd_vel_serial::~cmd_vel_serial()
{
    int closefd = close(serialfd);
}

void cmd_vel_serial::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    char serialheader = 0xf2;
    serialdata[0] = msg->linear.x;  //pitch
    serialdata[1] = msg->linear.y;  //roll
    serialdata[2] = msg->linear.z;  //height
    serialdata[3] = msg->angular.z; //yaw  

    int data_counter = write(serialfd, &serialheader, 1);
    data_counter = write(serialfd, serialdata, 16);
}


int cmd_vel_serial::set_term(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio;

    bzero(&newtio, sizeof(newtio));

    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    switch(nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    case 9:
        newtio.c_cflag |= CS8;
        break;
    }
    /* Set the Parity Bit */
    switch(nEvent)
    {
    case 'O': // Odd Parity
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP); // I'm not sure, but it doesn't matters
        break;
    case 'E': // Even Parity
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        newtio.c_cflag |= (INPCK | ISTRIP); // I'm not sure, but it doesn't matters
        break;
    case 'N': // None Parity
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        newtio.c_cflag &= ~PARENB;
        break;
    }
    /* Set the Baud Rate */
    switch(nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetispeed(&newtio, B19200);
        cfsetospeed(&newtio, B19200);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    /* Set the Stop Bit */
    if(nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if(nStop == 2)
        newtio.c_cflag |= CSTOPB;

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 12;

    tcflush(fd, TCIFLUSH);

    if((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }

    printf("set done!\n");
    return 0;
}

#endif

