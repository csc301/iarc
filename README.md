# iarc
this is for fun.......
 
            msg.linear_acceleration.x = pitch;       forward positive 
            msg.linear_acceleration.y = roll;          leftward positive
            msg.linear_acceleration.z = yaw;         Z positive

apt-get install package=version
   
    接收机         单片机捕捉             遥控器           单片机输出       NAZA      接收的变量      
      1                  inch1 PA8               roll                outch1 PC6          A          controlroll 
      2                  inch2 PA15             pitch             outch2 PC7          E          controlpitch  
      3                  inch3 PB4               throttle         outch3 PC8          T          controlthrottle
      4                  inch4 PB6               yaw               outch4 PC9          R          controlyaw

      6                  inch5 PA0               6


    sudo apt-get install libv4l.dev

Same here on openSUSE 11.1 for x86-64. Even kill -9 wouldn't kill the affected process. After half a day of system lockups, I solved the problem by uninstalling gvfs-fuse gvfs-backends packages and rebooting.

https://bugzilla.novell.com/show_bug.cgi?id=467862

sudo ln -sf ./libblas/libblas.so.3.0 ./libblas.so
sudo ln -sf ./lapack/liblapack.so.3.0 ./liblapack.so
 
    serialdata[0] = msg->linear.x;  //pitch            controlpitch         
    serialdata[1] = msg->linear.y;  //roll               controlroll     
    serialdata[2] = msg->linear.z;  //height          controlthrottle
    serialdata[3] = msg->angular.z; //yaw            controlyaw

    
rmmod ftdi_sio 

float32 ground_distance  # distance to ground in meters
int16   flow_x           # x-component of optical flow in pixels
int16   flow_y           # y-component of optical flow in pixels
float32 velocity_x       # x-component of scaled optical flow in m/s
float32 velocity_y       # y-component of scaled optical flow in m/s
uint8   quality          # quality of optical flow estimate   0-255

/lib/modules/3.16.0-37-generic/kernel/drivers/net/wireless
