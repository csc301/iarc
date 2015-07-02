#!/bin/sh -e

export ROS_MASTER_URI=http://pelican:11311

chmod 777 /dev/ttyUSB0 /dev/ttyACM0 /dev/video0
chmod 777 /dev/ttyUSB1 
rmmod ftdi_sio
chmod 777 /dev/bus/usb/002/006
chmod 777 /dev/bus/usb/002/010

