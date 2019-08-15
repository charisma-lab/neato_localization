#!/usr/bin/env python

# Module to track fiducial markers

import roslib
import rospy

from geometry_msgs.msg import Twist, Pose, PoseStamped, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import sys, select, termios, tty

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from neato_localization_laban.msg import NumPoints

from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy

import numpy as np
import cv2
import cv2.aruco as aruco
import json
import math
import time
import csv
import sys
import glob

# Maximum number of robots in the scene
MAX_BOTS = 2
# 0 -> in-built camera, 1 -> external USB webcam
VIDEO_SOURCE_ID = 1
WAIT_TIME = 1


class Tracker():
	def __init__(self):
		self._cap = cv2.VideoCapture(VIDEO_SOURCE_ID)
		self._dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) 
		self._font = cv2.FONT_HERSHEY_SIMPLEX
		rospy.Subscriber("/neato01/pose", PoseStamped, self.tracked_neato01, queue_size=10)
		cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
		cv2.resizeWindow('frame', 1280, 720)
		self._start_time = time.time()

	def track_every_frame(self):
		ret, frame = self._cap.read()
		colored_frame = frame
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		parameters =  aruco.DetectorParameters_create()
		font = cv2.FONT_HERSHEY_SIMPLEX

		self._detected_markers_in_this_frame = aruco.detectMarkers(colored_frame, self._dictionary, parameters=parameters)
		corners, ids, rejectedImgPoints = self._detected_markers_in_this_frame

		if len(self._detected_markers_in_this_frame[0]) > 0:
			for (fids, index) in zip(self._detected_markers_in_this_frame[0], self._detected_markers_in_this_frame[1]):
				for pt in fids:
					try:
						index_number = int(index[0])
						marker = NumPoints()
						marker.id = index_number
						points_list = []
						for point in pt:
							points_list.extend(list(point))
						marker.points = points_list
						tracking_all_markers.publish(marker)
						if time.time() - self._start_time > 2:
							if index_number == 1:
								cv2.putText(frame, "Neato: " + str(1) + " " + str(self._neato01_pose), (0,64), font, 1, (100,0,200),2,cv2.LINE_AA)
					except IndexError:
						pass

		if len(self._detected_markers_in_this_frame[0]) > 0:
			aruco.drawDetectedMarkers(colored_frame, self._detected_markers_in_this_frame[0], self._detected_markers_in_this_frame[1])
		
		cv2.imshow('frame', colored_frame)
		if cv2.waitKey(WAIT_TIME) & 0xFF == ord('q'):
			self._cap.release()
			cv2.destroyAllWindows()
			sys.exit()

	def tracked_neato01(self, m):
		self._neato01_pose = (round(m.pose.position.x, 2), round(m.pose.position.y, 2), round(((euler_from_quaternion([m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w]))[-1])*180/np.pi, 2))


if __name__ == "__main__":
	rospy.init_node('track_aruco_markers', anonymous=True)
	tracking_all_markers = rospy.Publisher("/tracked_all_markers", NumPoints, queue_size=10)
	watch_dogs = Tracker()
	while (True):
	  watch_dogs.track_every_frame()
