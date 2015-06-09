#!/usr/bin/env python
import numpy as np
#from random import randint
import rospy
#from std_msgs.msg import Header
#from geometry_msgs.msg import PointStamped,Point,Polygon,PolygonStamped,Point32
from muilt_object.msg import RoiArray
from sensor_msgs.msg import RegionOfInterest, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.cv as cv
import copy

class region:
	def __init__(self, RegionOfInterest, tag):
		self.roi = RegionOfInterest
		#self.x = RegionOfInterest.x_offset
		#self.y = RegionOfInterest.y_offset
		#self.width = RegionOfInterest.width
		#self.height = RegionOfInterest.height
		self.tag = tag
		#self.color = (randint(0,255), randint(0,255), randint(0,255))


class predictor:
	def __init__(self):
		rospy.init_node('prediction')
		#self.pub_prediction = rospy.Publisher('prediction_result',PolygonStamped)
		self.sub_rois = rospy.Subscriber("rois_list", RoiArray, self.rois_callback)
		self.sub_image = rospy.Subscriber("usb_camera/image_rect_color", Image, self.image_callback)

		self.tag=1
		self.prediction = {}
		self.last_match = {}
		self.match = {}
		self.rois_arr=[]

		self.bridge = CvBridge()
		self.frame=0
		cv2.namedWindow("prediction",cv2.CV_WINDOW_AUTOSIZE)

		rospy.on_shutdown(self.cleanup)
		
	def image_callback(self, ros_image):
		#print "image_callback"
		frame = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
		self.frame = np.array(frame, dtype=np.uint8)		

	def rois_callback(self, rois_arr):
		#print "rois rois_callback"
		self.rois_arr = rois_arr.list
		print self.rois_arr
		self.last_match = copy.deepcopy(self.match)
		if len(self.rois_arr) > len(self.prediction):
			for region_pre in self.prediction.iteritems():
				self.match[region_pre[0]].roi = self.compare(region_pre[1])
			for roi in self.rois_arr:
				self.match[self.tag] = region(roi, self.tag)
				self.last_match[self.tag] = region(roi, self.tag)
				#print self.match
				self.tag += 1 
		else:
			for roi in self.rois_arr:
				select_key = self.select_key(roi)
				self.match[select_key].roi = roi
				self.prediction.pop(select_key)
			for region_key in self.prediction.iterkeys():
				self.match.pop(region_key)
				self.last_match.pop(region_key)
			self.prediction = {}
		#print "rois_callback 2"
		for match in self.match.iteritems():
			print match[0]
			self.prediction[match[0]] = self.prediction_generate(match[1].roi, self.last_match[match[0]].roi)
			cv2.rectangle(self.frame, (match[1].roi.x_offset, match[1].roi.y_offset), (match[1].roi.x_offset+match[1].roi.width, match[1].roi.y_offset+match[1].roi.height), [0, 255, 0])
			cv2.putText(self.frame, str(match[0]), (match[1].roi.x_offset, match[1].roi.y_offset), cv2.FONT_HERSHEY_COMPLEX, 0.7, [0,255,0])
		#print "rois_callback 3"
		#print self.match
		#print self.last_match
		#print self.prediction
		cv2.imshow("prediction", self.frame)
		#print "rois_callback 4"
		cv.WaitKey(33)

	def compare(self, roi_pre):
		#print "compare"
		distance={}
		i=0
		for candidate in self.rois_arr:
			distance[i] = (candidate.x_offset - roi_pre.x_offset)**2 + (candidate.y_offset - roi_pre.y_offset)**2
			i=i+1
		distance=sorted(distance.iteritems(),key=lambda x:x[1])
		print distance
		return self.rois_arr.pop(distance[0][0])

	def select_key(self, roi):
		#distance= (self.prediction.iteritems[0][1].x_offset - roi.x_offset)^2 + (self.prediction.iteritems[0][1].y_offset - roi.y_offset)^2
		#i=0
		#print "select_key"
		distance = {}
		for candidate in self.prediction.iteritems():
			distance[candidate[0]] = (candidate[1].x_offset - roi.x_offset)**2 + (candidate[1].y_offset - roi.y_offset)**2
		#	i=i+1
		distance=sorted(distance.iteritems(),key=lambda x:x[1])
		print distance
		return distance[0][0]	

	def prediction_generate(self, match, last_match):
		#print "prediction_generate"
		prediction_roi = RegionOfInterest()
		prediction_roi.x_offset = 2 * match.x_offset - last_match.x_offset
		prediction_roi.y_offset = 2 * match.y_offset - last_match.y_offset
		print match.x_offset, last_match.x_offset, prediction_roi.x_offset
		print match.y_offset, last_match.y_offset, prediction_roi.y_offset
		return prediction_roi
		#temperarily discard the width and height of roi 

	def cleanup(self):
		print "Shutting down the predictor"
		print self.tag
		print self.prediction
		print self.match
		print self.last_match
		cv2.destroyAllWindows()


if __name__ == '__main__':
	try:
		predictor()
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down the predictor"
		cv2.DestroyAllWindows()






