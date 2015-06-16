#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped,Point,Polygon,PolygonStamped,Point32
from muilt_object.msg import RoiArray
from sensor_msgs.msg import RegionOfInterest


def compare(k_1,p,dete_result,radius=0.25):
	distance={}
	i=0
	dete_result=dete_result.reshape([10,2])
	for candidate in dete_result:
		distance[i]=np.dot(candidate,candidate)
		if distance[i]<radius:
			return candidate
	#prediction fails
	distance=sorted(distance.iteritems(),key=lambda x:x[1])
	return dete_result[distance[0][0]]
    
rospy.init_node('prediction')
pub_real = rospy.Publisher('real_position', PointStamped)
pub_prediction = rospy.Publisher('prediction_result',PolygonStamped)
r = rospy.Rate(100)

a=np.loadtxt(r"robo_dataset.txt")
robo1_real=a[:,[0,1]]
robo1_simu_temp=[]
for i in xrange(0,a.shape[0]):
	if i==0:
		k=a[i,[0,1]]
		p=a[i,[0,1]]
		robo1_simu_temp.append([k[0],k[1]])
		pub_real.publish(PointStamped(Header(frame_id="base_link",stamp=rospy.Time()),Point(k[0],k[1],0)))
		pub_prediction.publish(PolygonStamped(Header(frame_id="base_link",stamp=rospy.Time()),Polygon([Point32(p[0],p[1],0),Point32(0,0,0),Point32(10,10,0)])))
		r.sleep()
		continue
	k_1=k
	k=compare(k_1,p,a[i])
	p=k+k-k_1
	robo1_simu_temp.append([k[0],k[1]])
	pub_real.publish(PointStamped(Header(frame_id="base_link",stamp=rospy.Time()),Point(k[0],k[1],0)))
	pub_prediction.publish(PolygonStamped(Header(frame_id="base_link",stamp=rospy.Time()),Polygon([Point32(p[0],p[1],0),Point32(0,0,0),Point32(10,10,0)])))
	r.sleep()
robo1_simu=np.array(robo1_simu_temp)
delta=robo1_simu-robo1_real
total_shift=0
for x in delta:
	total_shift+=np.dot(x,x)
print total_shift



