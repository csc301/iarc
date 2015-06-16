# iarc
this is for fun.......
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

export ROS_MASTER_URI=http://pelican:11311
sudo rmmod ftdi_sio
ycc@E431:~/test_github$ ssh-keygen -C '1007081114@qq.com' -t rsa
Generating public/private rsa key pair.
Enter file in which to save the key (/home/ycc/.ssh/id_rsa): 
Enter passphrase (empty for no passphrase): 
Enter same passphrase again: 
Your identification has been saved in /home/ycc/.ssh/id_rsa.
Your public key has been saved in /home/ycc/.ssh/id_rsa.pub.
The key fingerprint is:
ff:cd:82:ce:82:17:c5:97:9c:6f:71:10:13:de:04:67 1007081114@qq.com
The key's randomart image is:
+--[ RSA 2048]----+
|              =+E|
|             ..* |
|         . . o...|
|          o = . .|
|        S. . . o |
|        ..    o  |
|       . ... .   |
|      . o....o   |
|       . oo ..o  |
+-----------------+
ycc@E431:~/test_github$ 


Header header

float32 ground_distance  # distance to ground in meters
int16   flow_x           # x-component of optical flow in pixels
int16   flow_y           # y-component of optical flow in pixels
float32 velocity_x       # x-component of scaled optical flow in m/s
float32 velocity_y       # y-component of scaled optical flow in m/s
uint8   quality          # quality of optical flow estimate   0-255



/lib/modules/3.16.0-37-generic/kernel/drivers/net/wireless



sudo apt-get install ros-indigo-turtlebot-* \
ros-indigo-openni-camera ros-indigo-openni-launch \
ros-indigo-openni-tracker ros-indigo-laser-* \
ros-indigo-audio-common ros-indigo-joystick-drivers \
ros-indigo-orocos-kdl ros-indigo-python-orocos-kdl \
ros-indigo-dynamixel-motor-* ros-indigo-pocketsphinx \
gstreamer0.10-pocketsphinx python-setuptools python-rosinstall \
ros-indigo-opencv2 ros-indigo-vision-opencv \
ros-indigo-depthimage-to-laserscan ros-indigo-arbotix-* \
git subversion mercurial

