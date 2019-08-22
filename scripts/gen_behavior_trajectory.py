#!/usr/bin/env python

'''
This node will generate the higher level trajectory for robot motion
using Laban Motion Features
'''

import rospy
import numpy as np
import math
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from std_msgs.msg import Header, Bool
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt

def create_behavior_dict(space, interaction, flow, time, weight):
    behavior = {'SPACE': space, 'INTERACTION': interaction, 'FLOW': flow, 'TIME': time, 'WEIGHT': weight}
    return behavior


def create_header(frame_id):
    #Q: What is this for?
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return header


def quat_heading(yaw):
    #Q: What is yaw?
    return Quaternion(*quaternion_from_euler(0, 0, yaw))


class BehaviorGenerator:
    def __init__(self):

        self.waypoint_publisher = rospy.Publisher('/neato01/social_global_plan', Path, queue_size=1)
        self.goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.changed_goal_publisher = rospy.Publisher('/neato01/changed_goal', Bool, queue_size=1)
        #Assumptions: neato05 is the person, neato01 is the robot
        #Q: What is the obstacle?
        rospy.Subscriber('/neato05/pose', PoseStamped, self.goal_callback, queue_size=1) # Use of service could be more efficient
        rospy.Subscriber('/neato01/pose', PoseStamped, self.start_callback, queue_size=1)
        rospy.Subscriber('/obstacle', PoseStamped, self.obstacle_callback, queue_size=1)

        self.start = None
        self.goal = None
        self.obstacle = None
        self.last_goal = None

        #Assumption for goal_change_threshold: how much the robot can steer away from the path, we can use this to manipulate FLOW
        self.goal_change_threshold = 0.3
        self.happy_section_length = 1.5
        self.sine_amplitude = 0.35
        self.change_goal_flag = True #Assumption: whether the goal has changed bc the person has moved
        self.start_goal_flag = True #Assumption: whether the robot started approaching the goal
        self.goal_to_publish = None  # TODO: redundant if i use path_to_publish for same purpose
        self.path_to_publish = None

    def goal_callback(self, goal_msg):
        """Assumption: callback on the location of the person"""
        # extract the pose and update
        self.goal = [goal_msg.pose.position.x, goal_msg.pose.position.y]
        self.goal_stamped = goal_msg

        if self.start_goal_flag:
            self.last_goal = self.goal #Assumption: set new goal

        #print('self.goal : ', self.goal)

    def start_callback(self, start_msg):
        """Assumption: callback on the location of the robot"""
        self.start = [start_msg.pose.position.x, start_msg.pose.position.y]
        #print('self.start : ', self.start)

    def obstacle_callback(self, obstacle_msg):
        #Q: When is this used and who publishes /obstacle topic?
        self.obstacle = obstacle_msg

    def populate_path_msg(self, rotated_path, heading):
        """Assumption: returns a list of all the points in the path"""
        poses_waypoints = []
        for i, point in enumerate(rotated_path):
            # print('rotated_path : ', rotated_path)
            # print('point : ', point)
            header = create_header('map')
            waypoint = Pose(Point(float(point[0]), float(point[1]), 0), quat_heading(float(heading[i])))
            poses_waypoints.append(PoseStamped(header, waypoint))
        return poses_waypoints

    def get_dist_theta_to_goal(self):
        y_diff = self.goal[1] - self.start[1]
        x_diff = self.goal[0] - self.start[0]
        theta = math.atan2(y_diff, x_diff) #Q: Why are we calculating this angle?
        dist = math.sqrt(x_diff ** 2 + y_diff ** 2) #distance between two points
        return x_diff, y_diff, dist, theta

    def get_change_goal_dist(self):
        y_diff = self.goal[1] - self.last_goal[1]
        x_diff = self.goal[0] - self.last_goal[0]
        dist = math.sqrt(x_diff ** 2 + y_diff ** 2)
        return dist

    def sample_poses(self, dist, theta, intermediate_steps):
        # fix equidistant points on x-axis
        sample_x = np.array(np.linspace(0, dist, num=intermediate_steps))
        sample_y = np.random.uniform(-1.0, 1.0, intermediate_steps)
        return sample_x, sample_y

    def gen_heading(self, path_rotated):
        """Take path, and generate heading (in euler) based on the slope of point[i] to point[i+1]"""
        # path_rotated has the start and end points
        heading = np.zeros([path_rotated.shape[0], 1])
        # Find heading for each point
        for i in range(len(path_rotated) - 1): # since we are calculating dx, dy wrt i, and i+1
            dx = path_rotated[i + 1, 0] - path_rotated[i, 0]
            dy = path_rotated[i + 1, 1] - path_rotated[i, 1]
            heading[i] = math.atan2(dy, dx)
        heading[-1] = heading[-2]  # TODO: Or make it zero if you want
        return heading

    def gen_traj_section(self, wavelength, amp, wave_formed,freq):
        """
        @param self
        @param wavelength
        @param amp
        @param wave_formed
        @param freq
        @return section_path which is an array
        """
        section_path = np.linspace(wave_formed, wave_formed+wavelength, num=wavelength*50)
        # linspace returns evenly spaced numbers over a specified interval
        omega = 2*np.pi*freq
        section_path = np.array([[L, amp * np.sin(omega * L)] for L in section_path])
        # Q: What does the above line of code do?
        return section_path

    def gen_traj_section_last(self, wavelength, amp, wave_formed, freq):
        omega = 2 * np.pi * freq
        #section_path = np.linspace(wave_formed, wave_formed+wavelength, num=wavelength*50)
        section_path = np.linspace(0, wavelength, num=wavelength * 50)
        section_path = np.array([[L+wave_formed, amp * np.sin(omega * L)] for L in section_path])
        return section_path

    def check_goal_change(self):
        """Assumption: check_goal_change sees if the goal has moved more than the
        the goal threshold before it is registered as a change in the person's position
        and a new path will be generated"""
        goal_change_dist = self.get_change_goal_dist()
        has_changed = (abs(goal_change_dist) > self.goal_change_threshold)
        return has_changed

    def manipulate_space(self, space_setting):
        '''
        SPACE is manipulated by altering the amplitude that the path takes
        takes in the space setting and decides how the path is manipulated to suit this setting

        @param space_setting
        @param path is a list of waypoints (something like this)

        '''

        amplitude = space_setting

        return amplitude


    def manipulate_interaction(self, interaction_setting):
        '''
        INTERACTION is manipulated by altering the end goal of the path
        takes in the interaction setting and decides how the path is manipulated to suit this setting

        @param interaction_setting

        '''
        self.last_goal = self.goal
        if interaction_setting == 1:
            self.last_goal[0] = 3.0 - self.last_goal[0]
            self.last_goal[1] = 2.5 - self.last_goal[1]



    def manipulate_time(self, time_setting, wavelength):
        '''
        TIME is manipulated by changing the frequency of the wave
        @param time_setting =
        @param path =

        '''
        freq = time_setting/wavelength
        return freq


    def generate_path(self, space_in, time_in, interact_in):
        '''
        @param laban_settings is a dictionary of the laban efforts created by create_behavior_dict
        @return path is a list of waypoints (something like this)
        call manipulate_space and manipulate_interaction based on the dictionary of settings
        '''


        # first we want to manipulate the interaction variable but only if the goal has changed
        if self.check_goal_change() or self.start_goal_flag:
            # get the interaction setting, either seeking, avoiding or neutral
            self.manipulate_interaction(interact_in)

            x_diff, y_diff, dist, theta = self.get_dist_theta_to_goal()

            wavelength = dist

            path = []

        # get the space setting

            space = self.manipulate_space(space_in)

        # if this is all we do maybe we can just have it done in here but also whatever

        # now we have path that has space yay

        # now we want to add the time and whatever
        # then we pass to
        # manipulate_time which will do stuff with the frequency yay
            time = self.manipulate_time(time_in, wavelength)
        # then we finish up whatever we need to do to make the path work and publish



            path = self.gen_traj_section(wavelength, space*0.5, 0, time)
            # Define rotation matrix to rotate the whole line
            rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
                # Q: Does this make an array of tuples of lists?


            path_rotated = np.zeros([path.shape[0], path.shape[1]])
            heading = np.zeros([path.shape[0],1])

                # loop simultaneously
            for i, point in enumerate(path):
                result = np.matmul(rot, point) #matrix product of two arrays
                path_rotated[i] = [result[0] + self.start[0], result[1] + self.start[1]]
                    # Now, path is from start point to goal point
                path_rotated[0] = self.start # TODO: check if this is coming out correctly
                path_rotated[-1] = self.goal

                # Find heading for each point #TODO: replace this with gen_heading?
            for i in range(len(path_rotated)-1):
                dx = path_rotated[i+1, 0] - path_rotated[i, 0]
                dy = path_rotated[i+1, 1] - path_rotated[i, 1]
                heading[i] = math.atan2(dy, dx)
                heading[-1] = heading[-2]

                # Now, populate the Path message #TODO: move to separate function
                path_to_publish = Path()
                path_to_publish.header = create_header('map')
                path_to_publish.poses = self.populate_path_msg(path_rotated, heading)
                self.waypoint_publisher.publish(path_to_publish)

                # goal_to_publish = MoveBaseActionGoal()
                # goal_to_publish.header = path_to_publish.header
                # goal_to_publish.goal.target_pose = self.goal_stamped

                self.path_to_publish = path_to_publish  # save it
                # self.goal_publisher.publish(self.goal_to_publish)
                print("changed goal")
                self.start_goal_flag = False

        else:
            self.waypoint_publisher.publish(self.path_to_publish)
            print("publishing old path")

#------------------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node("waypoint_publisher")
    print('Node : waypoint_publisher started')
    space_var = input("Enter the setting for SPACE from 0 (direct) to 5 (indirect): ")
    time_var = input("Enter the setting for TIME from 0 (abrupt) to 5 (sustained): ")
    interaction_var = input("Enter the setting for INTERACTION: 1) avoiding \t 2) seeking ")
    generator = BehaviorGenerator()
    r = rospy.Rate(20)
    start_time = time.time()
    while not rospy.is_shutdown():
        if time.time() - start_time > 1:
            generator.generate_path(space_var,time_var,interaction_var)
        r.sleep()
