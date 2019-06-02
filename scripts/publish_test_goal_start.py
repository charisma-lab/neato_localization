#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


def quat_heading(yaw):
    return Quaternion(*quaternion_from_euler(0, 0, yaw))


def create_header(frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return header


def publish(start, goal):
    start_pub.publish(start)
    goal_pub.publish(goal)
    print("Publishing start and goal points for testing.")


if __name__ == "__main__":
    rospy.init_node("pub_test_goal_start")
    start_pub = rospy.Publisher('/neato01/pose', PoseStamped, queue_size=1)
    goal_pub = rospy.Publisher('/neato05/pose', PoseStamped, queue_size=1)

    header = create_header('map')
    start_pose = Pose(Point(0.0, 0.0, 0.0), quat_heading(0.0))
    goal_pose = Pose(Point(4.0, 4.0, 0.0), quat_heading(0.0))

    start_pose_stampted = PoseStamped(header, start_pose)
    goal_pose_stampted = PoseStamped(header, goal_pose)

    print('Node : pub_test_goal_start started\n')

    r = rospy.Rate(5)
    start_time = time.time()
    while not rospy.is_shutdown():
        if time.time() - start_time > 1:
            publish(start_pose_stampted, goal_pose_stampted)
        r.sleep()
