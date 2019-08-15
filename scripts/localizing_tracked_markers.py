#!/usr/bin/env python

# Module to determine pose of tracked markers

import roslib
import rospy
import numpy as np
import time, math

from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from neato_localization_laban.msg import NumPoints
from tf.transformations import euler_from_quaternion, quaternion_from_euler


PIXEL_TO_WORLD_RATIO = 3.50
DISTANCE_CONSTANT_LONG = 3.50
DISTANCE_CONSTANT_CORNERS = .21
MAP_WIDTH = 2.50
REFERENCE_IDS = [0, 10]


class Marker():
	def __init__(self, marker):
		self._marker_id = marker.id
		# four corners -> [x0,y0,x1,y1,x2,y2,x3,y3,x4,y4]
		self._marker_points = marker.points
		if marker.id < 10:
			self._pub_pose = rospy.Publisher("/neato0"+str(self._marker_id)+"/pose", PoseStamped, queue_size=10)
		else:
			self._pub_pose = rospy.Publisher("/neato"+str(self._marker_id)+"/pose", PoseStamped, queue_size=10)

	def update_pixels(self, marker):
		self._marker_points = marker.points

	def update_pose(self, (ref_x, ref_y), delta_angle, PIXEL_TO_WORLD_RATIO):
		x = 0
		y = 0
		for counter, val in enumerate(self._marker_points):
			if counter % 2:
				y += val
			else:
				x += val
		x = x/4
		y = y/4
		distance_in_pixels = calculate_euc_distance((ref_x, ref_y), (x, y))
		distance_in_meters = distance_in_pixels*PIXEL_TO_WORLD_RATIO
		angle_transform = calculate_ang_deviation((ref_x, ref_y), (x, y))
		angle_transform = math.atan((-ref_y + y)/(ref_x - x + 0.0001))
		orientation = -1*calculate_ang_deviation((self._marker_points[4], self._marker_points[5]), (self._marker_points[2], self._marker_points[3]))
		new_x = (distance_in_meters*math.cos(angle_transform))
		new_y = (distance_in_meters*math.sin(angle_transform))
		orientation -= delta_angle
		# if self._marker_points[1] <= self._marker_points[-1]:
		# 	if self._marker_points[0] >= self._marker_points[-2]:
		# 		orientation -= delta_angle
		# 		orientation = orientation
		# 	else:
		# 		orientation = 1*orientation + np.pi
		# 		orientation += delta_angle
		# else:
		# 	if self._marker_points[0] >= self._marker_points[-2]:
		# 		orientation += delta_angle
		# 	else:
		# 		orientation = 1*orientation - np.pi
		# 		orientation -= delta_angle

		# print("3rd orientation : {}\n".format(orientation))
		# orientation = math.degrees(orientation)
		self._marker_pose = (new_x, new_y, orientation)
		#self._marker_pose = (1, 1.5, -.4)
		self.publish_pose()

	def publish_pose(self):
		pose_stamped = PoseStamped()
		pose_stamped.header.frame_id = 'map'
		pose_stamped.header.stamp = rospy.Time(0)
		pose_stamped.pose.position.x = self._marker_pose[0]
		pose_stamped.pose.position.y = self._marker_pose[1]
		pose_stamped.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self._marker_pose[2]))
		self._pub_pose.publish(pose_stamped)

	def get_pose(self):
		return self._marker_pose

	def get_corner(self, index):
		return (self._marker_points[index*2], self._marker_points[index*2 + 1])


class All_markers():
	def __init__(self):
		self._number_of_markers_detected = 0
		self._markers = dict()

	def create_marker(self, marker):
		# print(self._markers)
		if marker.id not in self._markers.keys():
			# create marker
			self._markers[marker.id] = Marker(marker)
			self._number_of_markers_detected = len(self._markers.keys())
		else:
			# update marker
			self._markers[marker.id].update_pixels(marker)

	def get_all_robot_poses(self):
		all_robot_poses = dict()
		for elem, each_marker in self._markers.items():
			if elem not in REFERENCE_IDS:
				all_robot_poses[elem] = each_marker.get_pose()
		# returns a dict of marker ids and the poses
		return all_robot_poses

	def estimate_robot_pose(self):
		if REFERENCE_IDS[0] in self._markers.keys() and REFERENCE_IDS[1] in self._markers.keys():
			self._distance_in_pixels = calculate_euc_distance(self._markers[REFERENCE_IDS[0]].get_corner(3), self._markers[REFERENCE_IDS[1]].get_corner(0))
			self._angle_deviation = calculate_ang_deviation(self._markers[REFERENCE_IDS[0]].get_corner(3), self._markers[REFERENCE_IDS[1]].get_corner(0))
			PIXEL_TO_WORLD_RATIO = DISTANCE_CONSTANT_LONG/self._distance_in_pixels
			for elem, each_marker in self._markers.items():
				# if elem not in REFERENCE_IDS:
				each_marker.update_pose(self._markers[REFERENCE_IDS[1]].get_corner(0), self._angle_deviation, PIXEL_TO_WORLD_RATIO)
		elif REFERENCE_IDS[1] in self._markers.keys():
		  self._distance_in_pixels = calculate_euc_distance(self._markers[REFERENCE_IDS[1]].get_corner(3), self._markers[REFERENCE_IDS[1]].get_corner(0))
		  self._angle_deviation = calculate_ang_deviation(self._markers[REFERENCE_IDS[1]].get_corner(3), self._markers[REFERENCE_IDS[1]].get_corner(0))
		  PIXEL_TO_WORLD_RATIO = DISTANCE_CONSTANT_CORNERS/self._distance_in_pixels
		  for elem, each_marker in self._markers.items():
		      each_marker.update_pose(self._markers[REFERENCE_IDS[1]].get_corner(0), self._angle_deviation, PIXEL_TO_WORLD_RATIO)
		elif REFERENCE_IDS[0] in self._markers.keys():
		  self._distance_in_pixels = calculate_euc_distance(self._markers[REFERENCE_IDS[0]].get_corner(3), self._markers[REFERENCE_IDS[0]].get_corner(0))
		  self._angle_deviation = calculate_ang_deviation(self._markers[REFERENCE_IDS[0]].get_corner(3), self._markers[REFERENCE_IDS[0]].get_corner(0))
		  PIXEL_TO_WORLD_RATIO = DISTANCE_CONSTANT_CORNERS/self._distance_in_pixels
		  for elem, each_marker in self._markers.items():
		      new_ref = self._markers[REFERENCE_IDS[0]].get_corner(0)
		      each_marker.update_pose((int(new_ref[0] - (DISTANCE_CONSTANT_LONG/PIXEL_TO_WORLD_RATIO)*cos(self._angle_deviation)), int(new_ref[1] - (DISTANCE_CONSTANT_LONG/PIXEL_TO_WORLD_RATIO)*sin(self._angle_deviation))), self._angle_deviation, PIXEL_TO_WORLD_RATIO)


def calculate_euc_distance((x0, y0), (x1, y1)):
	return math.sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1))

def calculate_ang_deviation((x0, y0), (x1, y1)):
	delta_x = x0 - x1 + 0.0001*0.0001
	delta_y = y0 - y1
	return math.atan2(delta_y, delta_x)

def tracked_marker_callback(marker):
	all_neato_robots.create_marker(marker)


if __name__ == "__main__":
	rospy.init_node('localize_and_find_pose', anonymous=True)
	global all_neato_robots
	all_neato_robots = All_markers()
	rospy.Subscriber("/tracked_all_markers", NumPoints, tracked_marker_callback)
	r = rospy.Rate(20)
	start_time = time.time()
	while not rospy.is_shutdown():
		all_neato_robots.estimate_robot_pose()
		if time.time() - start_time > 2:
			pass
			# print(all_neato_robots.get_all_robot_poses())
		r.sleep()
