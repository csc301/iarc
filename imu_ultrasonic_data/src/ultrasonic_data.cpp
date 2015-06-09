#include "ros/ros.h"
#include "std_msgs/Float32.h"
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

int set_term(int fd, int nSpeed, int nBits, char nEvent, int nStop);

int main(int argc,char **argv)
{
    ros::init(argc,argv,"ultrasonic_node");
    ros::NodeHandle n;
    ros::Publisher ultrasonic_pub = n.advertise<std_msgs::Float32>("ultrasonic_data",100);      
    ros::Rate loop_rate(30);  
    std_msgs::Float32 msg;
    
    std::string ultrasonic_serial_port;
    if (!n.getParam ("from_ultrasonic/ultrasonic_port", ultrasonic_serial_port))  
      ultrasonic_serial_port = "/dev/ttyUSB1";
	
    int i,serial_fd;
    float height;
    char read_buffer[4],order=0x55;;
    short* hei; 
    short  ht;
    struct termios oldtio;
    int serialfd = open(ultrasonic_serial_port.c_str (), O_RDWR);
    if(-1 == serialfd)
      {
        perror("open serial error!");
        exit(1);
      }
    if(tcgetattr(serialfd, &oldtio) != 0)
      {
        perror("SetupSerial");
        return -1;
      }
    if((i = set_term(serialfd, 9600, 8, 'N', 1)) < 0)
      {
        perror("set_term error");
        exit(1);
      }

    while(ros::ok())
      {
        int data_counter = write(serialfd, &order, 1);
        struct timeval serial_timeout;
        fd_set serial_readfds;
        FD_ZERO(&serial_readfds);
        FD_SET(serialfd, &serial_readfds);
        serial_timeout.tv_sec = 0;          // 0s   
        serial_timeout.tv_usec = 100000;    // 100ms
        int select_flag = select(FD_SETSIZE, &serial_readfds, (fd_set *)NULL, (fd_set *)NULL, &serial_timeout);
 
        switch(select_flag) 
          {
            case 0:
              i++;
	printf("serial receive error for %d times\n",i);
            break;
 
            case -1:
              perror("select");
              exit(1);

            default:
              for(serial_fd = 0; serial_fd < FD_SETSIZE; serial_fd++) 
                {
                  if(FD_ISSET(serial_fd, &serial_readfds))
                     {
                       data_counter = read(serial_fd, read_buffer, 2);
                       read_buffer[3]=read_buffer[0];
                       read_buffer[0]=read_buffer[1];
                       read_buffer[1]=read_buffer[3];
                       hei = (short*)(read_buffer);
                       ht=*hei;
                       height = (float)(ht/1000.0);  
                       msg.data = height; 
                        if(height<5.0)ultrasonic_pub.publish(msg); 
                     }
                }
            break;
          }
       ros::spinOnce();
       loop_rate.sleep();
     }
     int closefd = close(serialfd);
}


int set_term(int fd, int nSpeed, int nBits, char nEvent, int nStop)
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


