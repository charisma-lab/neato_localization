#!/usr/bin/env python

import roslib
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

import time
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy

# Module to track fiducial markers and output their pose

import numpy as np
import cv2
import cv2.aruco as aruco
import json
import math
import time
import csv
import sys

# Maximum number of robots in the scene
MAX_BOTS = 10
# 0 -> in-built camera, 1 -> external USB webcam
VIDEO_SOURCE_ID = 1
WAIT_TIME = 1


class Marker():
	def __init__(self):
		# position is (x,y)
		self._position = [0,0]
		# orientation is theta
		self._orientation = 0

	def update_pose(self, position, orientation):
		self._position = position
		self._orientation = orientation


class Tracker():
	def __init__(self):
		self._cap = cv2.VideoCapture(VIDEO_SOURCE_ID)
		self._dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
		self._all_black_track = create_blank(1920, 1080, rgb_color=(0, 0, 0))
		cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
		cv2.resizeWindow('frame', 1280, 720)

	def track_every_frame(self):
		ret, frame = self._cap.read()
		colored_frame = frame
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		parameters =  aruco.DetectorParameters_create()

		self._detected_markers_in_this_frame = aruco.detectMarkers(colored_frame, self._dictionary, parameters=parameters)
		corners, ids, rejectedImgPoints = self._detected_markers_in_this_frame
		font = cv2.FONT_HERSHEY_SIMPLEX

		# if np.all(ids != None):
			###### DRAW ID #####
			# cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

		if len(self._detected_markers_in_this_frame[0]) > 0:
			for (fids, index) in zip(self._detected_markers_in_this_frame[0], self._detected_markers_in_this_frame[1]):
				for pt in fids:
					try:
						if (int(index[0])==0):
							ll = ((pt[0] +pt[1] +pt[2] +pt[3])/4)
							cv2.circle(colored_frame,(ll[0],ll[1]), 15, (0,0,255), -1)
							cv2.circle(self._all_black_track,(ll[0],ll[1]), 15, (0,0,255), -1)
						elif (int(index[0])==1):
							ll = ((pt[0] +pt[1] +pt[2] +pt[3])/4)
							cv2.circle(colored_frame,(ll[0],ll[1]), 15, (0,255,0), -1)
							cv2.circle(self._all_black_track,(ll[0],ll[1]), 15, (0,255,0), -1)
						elif (int(index[0])==3):
							ll = ((pt[0] +pt[1] +pt[2] +pt[3])/4)
							cv2.circle(colored_frame,(ll[0],ll[1]), 15, (255,255,0), -1)
						elif (int(index[0])==2):
							ll = ((pt[0] +pt[1] +pt[2] +pt[3])/4)
							cv2.circle(colored_frame,(ll[0],ll[1]), 15, (255,0,0), -1)
							cv2.circle(self._all_black_track,(ll[0],ll[1]), 15, (255,0,0), -1)
					except IndexError:
						pass

		if len(self._detected_markers_in_this_frame[0]) > 0:
			aruco.drawDetectedMarkers(colored_frame, self._detected_markers_in_this_frame[0], self._detected_markers_in_this_frame[1])
		
		cv2.imshow('frame', colored_frame)
		if cv2.waitKey(WAIT_TIME) & 0xFF == ord('q'):
			self._cap.release()
			cv2.destroyAllWindows()
			sys.exit()


def create_blank(width, height, rgb_color=(0, 0, 0)):
	"""Create new image(numpy array) filled with certain color in RGB"""
	# Create black blank image
	image = np.zeros((height, width, 3), np.uint8)
	# Since OpenCV uses BGR, convert the color first
	color = tuple(reversed(rgb_color))
	# Fill image with color
	image[:] = color
	return image


if __name__ == "__main__":
	watch_dogs = Tracker()
	while (True):
		watch_dogs.track_every_frame()
