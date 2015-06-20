#!/bin/sh -e

sudo chmod 777 /dev/ttyUSB0 /dev/ttyACM0 /dev/video0
sudo rmmod ftdi_sio 
sudo chmod 777 /dev/ttyUSB1 
sudo chmod 777 /dev/bus/usb/002/006

export ROS_MASTER_URI=http://pelican:11311
